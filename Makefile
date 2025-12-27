# Makefile for Maqueen Plus V3 MicroPython Project
# CLI workflow for flashing, uploading, and debugging

# ========== Configuration ==========
SRC_DIR = src
EXAMPLES_DIR = examples

# Main program
MAIN = $(SRC_DIR)/main.py

# Library files to upload to micro:bit filesystem
LIBS = $(SRC_DIR)/maqueen_plus_v3.py \
       $(SRC_DIR)/laser_matrix.py

# Utility scripts
I2C_SCANNER = $(SRC_DIR)/i2c_scanner.py

# Auto-detect micro:bit serial port
# Common paths: /dev/ttyACM0, /dev/ttyUSB0
PORT := $(shell ls /dev/ttyACM* 2>/dev/null | head -n1)
ifeq ($(PORT),)
	PORT := $(shell ls /dev/ttyUSB* 2>/dev/null | head -n1)
endif
ifeq ($(PORT),)
	PORT = /dev/ttyACM0
endif

# Serial settings
BAUD = 115200

# Terminal program (screen or picocom)
TERM := $(shell which picocom 2>/dev/null)
ifeq ($(TERM),)
	TERM := $(shell which screen 2>/dev/null)
	TERM_ARGS = $(PORT) $(BAUD)
else
	TERM_ARGS = $(PORT) -b $(BAUD)
endif

# ========== Phony Targets ==========
.PHONY: all flash upload upload-libs clean repl scan test help install-deps

# ========== Main Targets ==========

# Default target: flash and upload everything
all: flash upload
	@echo ""
	@echo "✓ All files deployed!"
	@echo "  Press the reset button on micro:bit to run the program"
	@echo "  Or run 'make repl' to access the REPL"

# Flash main.py to micro:bit (replaces entire program)
flash: $(MAIN)
	@echo "Flashing main.py to micro:bit..."
	@if [ ! -f "$(MAIN)" ]; then \
		echo "Error: $(MAIN) not found"; \
		exit 1; \
	fi
	uflash $(MAIN)
	@echo "✓ Flash complete!"

# Upload library files to micro:bit filesystem
upload: upload-libs
	@echo "✓ All libraries uploaded!"

upload-libs: $(LIBS)
	@echo "Uploading libraries to micro:bit filesystem..."
	@for lib in $(LIBS); do \
		if [ -f "$$lib" ]; then \
			echo "  Uploading $$lib..."; \
			ufs put "$$lib"; \
		else \
			echo "  Warning: $$lib not found"; \
		fi \
	done

# Upload specific example
upload-example:
	@if [ -z "$(FILE)" ]; then \
		echo "Usage: make upload-example FILE=examples/line_follower.py"; \
		exit 1; \
	fi
	@if [ ! -f "$(FILE)" ]; then \
		echo "Error: $(FILE) not found"; \
		exit 1; \
	fi
	@echo "Uploading $(FILE)..."
	ufs put "$(FILE)"
	@echo "✓ Uploaded!"

# List files on micro:bit
ls:
	@echo "Files on micro:bit:"
	ufs ls

# Clean files from micro:bit
clean:
	@echo "Removing files from micro:bit..."
	-ufs rm maqueen_plus_v3.py
	-ufs rm laser_matrix.py
	@echo "✓ Cleanup complete!"

# Hard clean: remove all files
clean-all: clean
	@echo "WARNING: This will erase main.py!"
	@read -p "Continue? [y/N] " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		ufs rm main.py; \
		echo "✓ All files removed!"; \
	fi

# ========== Debugging & Testing ==========

# Open serial REPL
repl:
	@echo "Opening REPL on $(PORT)..."
	@echo "Press Ctrl-A then K to exit screen, or Ctrl-A then X for picocom"
	@if [ ! -e "$(PORT)" ]; then \
		echo "Error: micro:bit not found at $(PORT)"; \
		echo "Available ports:"; \
		ls -1 /dev/tty* | grep -E "(ACM|USB)"; \
		exit 1; \
	fi
	@sleep 1
	$(TERM) $(TERM_ARGS)

# Run I2C scanner
scan:
	@echo "Flashing I2C scanner..."
	uflash $(I2C_SCANNER)
	@echo "Scanner flashed! Connect to REPL to see results:"
	@echo "  make repl"

# Test laser sensor
test-laser:
	@echo "Flashing laser sensor test..."
	uflash $(SRC_DIR)/test_laser.py
	@echo "Uploading laser library..."
	ufs put $(SRC_DIR)/laser_matrix.py
	@echo "Test deployed! Connect to REPL to see results:"
	@echo "  make repl"

# Run self-test
test: flash upload
	@echo "Self-test deployed!"
	@echo "Press button A on micro:bit to start self-test"
	@echo "Connect to REPL to see results:"
	@echo "  make repl"

# ========== Deployment Shortcuts ==========

# Deploy obstacle avoidance (default program)
obstacle: all
	@echo "Obstacle avoidance program deployed!"

# Deploy line follower example
line-follower:
	@echo "Deploying line follower..."
	uflash $(EXAMPLES_DIR)/line_follower.py
	$(MAKE) upload-libs
	@echo "✓ Line follower deployed!"

# Deploy light seeker example
light-seeker:
	@echo "Deploying light seeker..."
	uflash $(EXAMPLES_DIR)/light_seeker.py
	$(MAKE) upload-libs
	@echo "✓ Light seeker deployed!"

# Deploy remote control example
remote-control:
	@echo "Deploying remote control..."
	uflash $(EXAMPLES_DIR)/remote_control.py
	$(MAKE) upload-libs
	@echo "✓ Remote control deployed!"

# ========== Installation ==========

# Install required tools
install-deps:
	@echo "Installing dependencies..."
	@echo ""
	@echo "Required Python packages:"
	pip install --user uflash microfs
	@echo ""
	@echo "Required system packages:"
	@if command -v apt-get >/dev/null 2>&1; then \
		echo "Run: sudo apt-get install screen picocom"; \
	elif command -v pacman >/dev/null 2>&1; then \
		echo "Run: sudo pacman -S screen picocom"; \
	else \
		echo "Install screen or picocom using your package manager"; \
	fi
	@echo ""
	@echo "✓ Python packages installed!"

# Check installation
check:
	@echo "Checking installation..."
	@echo ""
	@echo -n "uflash: "
	@which uflash >/dev/null 2>&1 && echo "✓ installed" || echo "✗ missing (run: pip install uflash)"
	@echo -n "microfs: "
	@which ufs >/dev/null 2>&1 && echo "✓ installed" || echo "✗ missing (run: pip install microfs)"
	@echo -n "screen: "
	@which screen >/dev/null 2>&1 && echo "✓ installed" || echo "○ optional"
	@echo -n "picocom: "
	@which picocom >/dev/null 2>&1 && echo "✓ installed" || echo "○ optional"
	@echo ""
	@echo -n "micro:bit: "
	@if [ -e "$(PORT)" ]; then \
		echo "✓ found at $(PORT)"; \
	else \
		echo "✗ not found"; \
		echo "  Available ports:"; \
		ls -1 /dev/tty* 2>/dev/null | grep -E "(ACM|USB)" || echo "  (none)"; \
	fi

# ========== Help ==========

help:
	@echo "Maqueen Plus V3 - MicroPython Makefile"
	@echo ""
	@echo "Development Commands:"
	@echo "  make all          - Flash main.py and upload libraries (default)"
	@echo "  make flash        - Flash main.py to micro:bit"
	@echo "  make upload       - Upload library files to filesystem"
	@echo "  make ls           - List files on micro:bit"
	@echo "  make clean        - Remove library files from micro:bit"
	@echo "  make clean-all    - Remove all files including main.py"
	@echo ""
	@echo "Debugging:"
	@echo "  make repl         - Open serial REPL (exit: Ctrl-A K)"
	@echo "  make scan         - Run I2C scanner to identify components"
	@echo "  make test-laser   - Test laser ToF sensor (full diagnostic)"
	@echo "  make test         - Deploy and run self-test"
	@echo ""
	@echo "Examples:"
	@echo "  make obstacle     - Deploy obstacle avoidance (default)"
	@echo "  make line-follower   - Deploy line follower"
	@echo "  make light-seeker    - Deploy light seeker"
	@echo "  make remote-control  - Deploy remote control"
	@echo ""
	@echo "Setup:"
	@echo "  make install-deps - Install required tools"
	@echo "  make check        - Check installation"
	@echo ""
	@echo "Configuration:"
	@echo "  Port: $(PORT)"
	@echo "  Baud: $(BAUD)"
	@echo ""
