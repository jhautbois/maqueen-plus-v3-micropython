#!/usr/bin/env python3
"""Build script: create standalone micro:bit program.

Combines src/*.py into a single standalone.py, then minifies it.
No need to upload separate library files!

Usage: python build.py [--flash]
"""
import sys
import subprocess
import shutil
import re
from pathlib import Path

SRC_DIR = Path("src")
BUILD_DIR = Path("build")

# Order matters: dependencies first
LIB_FILES = ["laser_matrix.py", "maqueen_plus_v3.py"]
MAIN_FILE = "main.py"


def extract_imports_and_code(filepath: Path) -> tuple[set, list]:
    """Extract standard imports and code from a Python file."""
    content = filepath.read_text()
    lines = content.split('\n')

    imports = set()
    code_lines = []
    skip_until_class = False
    in_try_block = False

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Skip docstrings at file start
        if stripped.startswith('"""') or stripped.startswith("'''"):
            i += 1
            continue

        # Skip try/except ImportError blocks entirely
        if stripped == 'try:' and i + 1 < len(lines):
            next_line = lines[i + 1].strip()
            if 'from laser_matrix import' in next_line or 'import laser_matrix' in next_line:
                # Skip: try, import, except, LaserMatrix = None
                i += 4
                continue

        # Collect standard library imports
        if stripped.startswith('from microbit import') or \
           stripped.startswith('import neopixel') or \
           stripped.startswith('import music'):
            imports.add(stripped)
            i += 1
            continue

        # Skip local imports (we'll embed them)
        if stripped.startswith('from maqueen_plus_v3 import') or \
           stripped.startswith('from laser_matrix import') or \
           stripped.startswith('import maqueen_plus_v3') or \
           stripped.startswith('import laser_matrix'):
            i += 1
            continue

        code_lines.append(line)
        i += 1

    return imports, code_lines


def build_standalone() -> Path:
    """Combine all source files into a single standalone file."""
    print("Building standalone version...")

    BUILD_DIR.mkdir(exist_ok=True)

    all_imports = set()
    all_code = []

    # Process library files first
    for lib in LIB_FILES:
        lib_path = SRC_DIR / lib
        if lib_path.exists():
            imports, code = extract_imports_and_code(lib_path)
            all_imports.update(imports)
            all_code.append(f"\n# === {lib} ===")
            all_code.extend(code)
            print(f"  Added {lib}")

    # Process main file
    main_path = SRC_DIR / MAIN_FILE
    if main_path.exists():
        imports, code = extract_imports_and_code(main_path)
        all_imports.update(imports)
        all_code.append(f"\n# === {MAIN_FILE} ===")
        all_code.extend(code)
        print(f"  Added {MAIN_FILE}")

    # Combine into standalone file
    standalone_path = BUILD_DIR / "standalone.py"

    # Sort imports for consistency
    sorted_imports = sorted(all_imports)

    with open(standalone_path, 'w') as f:
        f.write('"""Standalone Obstacle Avoidance - Auto-generated"""\n')
        for imp in sorted_imports:
            f.write(imp + '\n')
        f.write('\n'.join(all_code))

    src_size = standalone_path.stat().st_size
    print(f"  Combined: {src_size} bytes")

    return standalone_path


def minify(src: Path) -> Path:
    """Minify a Python file."""
    dst = src.with_suffix('.min.py')

    try:
        result = subprocess.run(
            ["pyminify", "--remove-literal-statements", str(src)],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            dst.write_text(result.stdout)
            src_size = src.stat().st_size
            dst_size = dst.stat().st_size
            reduction = (1 - dst_size / src_size) * 100
            print(f"  Minified: {dst_size} bytes ({reduction:.0f}% smaller)")
            return dst
        else:
            print(f"  Minify failed: {result.stderr}")
            return src
    except FileNotFoundError:
        print("  WARNING: pyminify not found, using unminified version")
        return src


def flash(filepath: Path) -> bool:
    """Flash file to micro:bit using auto-mount script."""
    print(f"\nFlashing {filepath.name}...")

    # Try flash_microbit.sh first (handles mounting)
    flash_script = Path("flash_microbit.sh")
    if flash_script.exists():
        result = subprocess.run(["./flash_microbit.sh", str(filepath)])
        return result.returncode == 0

    # Fallback to uflash
    result = subprocess.run(["uflash", str(filepath)])
    return result.returncode == 0


def build():
    """Full build process."""
    # Clean
    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)

    # Combine sources
    standalone = build_standalone()

    # Minify
    minified = minify(standalone)

    # Copy as main.py for flashing
    final = BUILD_DIR / "main.py"
    shutil.copy(minified, final)

    print(f"\nBuild complete! {final}")
    return final


def main():
    args = sys.argv[1:]

    if "--help" in args or "-h" in args:
        print(__doc__)
        print("Options:")
        print("  (no args)  Build standalone version only")
        print("  --flash    Build and flash to micro:bit")
        return

    final = build()

    if "--flash" in args:
        if not flash(final):
            sys.exit(1)

    print("\nDone!")


if __name__ == "__main__":
    main()
