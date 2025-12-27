#!/usr/bin/env python3
"""Build script: minify Python files for micro:bit deployment.

Keeps source files readable, creates minified versions in build/ directory.
Usage: python build.py [--flash] [--upload]
"""
import os
import sys
import subprocess
import shutil
from pathlib import Path

SRC_DIR = Path("src")
BUILD_DIR = Path("build")
MAIN_FILE = "main.py"
LIB_FILES = ["maqueen_plus_v3.py", "laser_matrix.py"]

def minify_file(src: Path, dst: Path) -> bool:
    """Minify a Python file using pyminify."""
    try:
        result = subprocess.run(
            ["pyminify", "--remove-literal-statements", str(src)],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_text(result.stdout)
            src_size = src.stat().st_size
            dst_size = dst.stat().st_size
            reduction = (1 - dst_size / src_size) * 100
            print(f"  {src.name}: {src_size} -> {dst_size} bytes ({reduction:.0f}% smaller)")
            return True
        else:
            print(f"  ERROR minifying {src}: {result.stderr}")
            return False
    except FileNotFoundError:
        print("ERROR: pyminify not found. Install with: pip install python-minifier")
        return False

def build():
    """Build minified versions of all source files."""
    print("Building minified files...")

    # Clean build directory
    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)
    BUILD_DIR.mkdir()

    success = True

    # Minify main file
    src = SRC_DIR / MAIN_FILE
    if src.exists():
        success &= minify_file(src, BUILD_DIR / MAIN_FILE)
    else:
        print(f"  WARNING: {src} not found")

    # Minify library files
    for lib in LIB_FILES:
        src = SRC_DIR / lib
        if src.exists():
            success &= minify_file(src, BUILD_DIR / lib)
        else:
            print(f"  WARNING: {src} not found")

    if success:
        print(f"\nBuild complete! Files in {BUILD_DIR}/")
    return success

def flash():
    """Flash main.py to micro:bit."""
    main_file = BUILD_DIR / MAIN_FILE
    if not main_file.exists():
        print("Build first!")
        return False

    print(f"\nFlashing {main_file}...")
    result = subprocess.run(["uflash", str(main_file)])
    return result.returncode == 0

def upload():
    """Upload library files to micro:bit filesystem."""
    print("\nUploading libraries...")
    for lib in LIB_FILES:
        lib_file = BUILD_DIR / lib
        if lib_file.exists():
            print(f"  Uploading {lib}...")
            result = subprocess.run(["ufs", "put", str(lib_file)])
            if result.returncode != 0:
                print(f"  ERROR uploading {lib}")
                return False
    return True

def main():
    args = sys.argv[1:]

    if "--help" in args or "-h" in args:
        print(__doc__)
        print("Options:")
        print("  (no args)  Build minified files only")
        print("  --flash    Build and flash main.py")
        print("  --upload   Build and upload libraries")
        print("  --all      Build, flash, and upload")
        return

    # Always build first
    if not build():
        sys.exit(1)

    if "--all" in args:
        if not flash():
            sys.exit(1)
        if not upload():
            sys.exit(1)
    else:
        if "--flash" in args:
            if not flash():
                sys.exit(1)
        if "--upload" in args:
            if not upload():
                sys.exit(1)

    print("\nDone!")

if __name__ == "__main__":
    main()
