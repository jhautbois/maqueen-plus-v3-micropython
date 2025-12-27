"""Maqueen Plus V3 - IR Remote Control Example
Control robot with infrared remote control

Note: This is a template implementation. The exact IR protocol
depends on your remote. You'll need to decode IR signals and
map them to commands.

Common IR remotes for Maqueen:
- DFRobot IR remote (comes with some kits)
- Generic TV remotes
- Arduino IR remotes

Controls:
- IR Remote: Various buttons for movement
- Button A: Enter learn mode
- Button B: Emergency stop
"""
from microbit import display, button_a, button_b, sleep, Image, pin8
from maqueen_plus_v3 import MaqueenPlusV3


# ========== Configuration ==========
DRIVE_SPEED = 150
TURN_SPEED = 120

# IR command codes (these are examples - adjust for your remote)
# You'll need to run in learn mode to discover your remote's codes
IR_COMMANDS = {
    # Arrow keys
    0x18: 'forward',      # Up arrow
    0x52: 'reverse',      # Down arrow
    0x08: 'turn_left',    # Left arrow
    0x5A: 'turn_right',   # Right arrow
    0x1C: 'stop',         # OK/Enter

    # Number keys (example)
    0x45: 'speed_up',     # +
    0x46: 'speed_down',   # -
    0x47: 'test',         # 0 - run self-test

    # Add your remote's codes here
}


class RemoteControl:
    """IR remote controlled robot"""

    def __init__(self):
        """Initialize robot"""
        print("Initializing Remote Control...")
        display.show(Image.HEART)

        self.robot = MaqueenPlusV3(laser_enabled=False)
        self.learn_mode = False
        self.speed = DRIVE_SPEED

        print("Ready!")
        print("Button A: Enter learn mode")
        print("Button B: Emergency stop")
        display.show(Image.HAPPY)

    def read_ir(self):
        """Read IR receiver

        Returns:
            IR code (int) or None if no signal

        Note: This is a simplified implementation.
        Actual IR reading on micro:bit requires more complex
        pulse timing analysis. Consider using:
        - NEC protocol decoder
        - External IR library
        - Polling STM8 if it handles IR decoding
        """
        # TODO: Implement actual IR reading
        # Options:
        # 1. Read from STM8 via I2C (if firmware supports it)
        # 2. Implement NEC/RC5 protocol decoder on pin8
        # 3. Use external IR library

        # Placeholder - read from STM8 register (adjust as needed)
        try:
            # This is speculative - check actual STM8 protocol
            data = self.robot._read_reg(0x40, 2)  # IR register (example)
            if data[0] != 0xFF:  # Check if valid code
                code = data[0]
                return code
        except:
            pass

        return None

    def execute_command(self, command):
        """Execute IR command

        Args:
            command: Command string from IR_COMMANDS
        """
        print(f"Command: {command}")

        if command == 'forward':
            self.robot.drive(self.speed)
            self.robot.headlights('green', 'green')
            display.show(Image.ARROW_N)

        elif command == 'reverse':
            self.robot.drive(-self.speed)
            self.robot.headlights('red', 'red')
            display.show(Image.ARROW_S)

        elif command == 'turn_left':
            self.robot.turn(-TURN_SPEED)
            self.robot.headlights('yellow', 'blue')
            display.show(Image.ARROW_W)

        elif command == 'turn_right':
            self.robot.turn(TURN_SPEED)
            self.robot.headlights('blue', 'yellow')
            display.show(Image.ARROW_E)

        elif command == 'stop':
            self.robot.stop()
            self.robot.headlights('off', 'off')
            display.show(Image.SQUARE)

        elif command == 'speed_up':
            self.speed = min(255, self.speed + 25)
            print(f"Speed: {self.speed}")
            display.scroll(str(self.speed), wait=False)

        elif command == 'speed_down':
            self.speed = max(50, self.speed - 25)
            print(f"Speed: {self.speed}")
            display.scroll(str(self.speed), wait=False)

        elif command == 'test':
            print("Running self-test...")
            self.robot.self_test()

        else:
            print(f"Unknown command: {command}")

    def learn_ir_codes(self):
        """Learn mode - display IR codes from remote

        Press buttons on remote to see their codes
        Use these to update IR_COMMANDS dictionary
        """
        print("\n" + "="*40)
        print("IR Learn Mode")
        print("Press buttons on remote to see codes")
        print("Press button A to exit")
        print("="*40 + "\n")

        display.scroll("LEARN", wait=False)

        while self.learn_mode:
            if button_a.was_pressed():
                self.learn_mode = False
                print("Exiting learn mode")
                display.show(Image.HAPPY)
                break

            code = self.read_ir()
            if code is not None:
                hex_code = "0x{:02X}".format(code)
                print(f"IR Code: {hex_code} ({code})")
                display.scroll(hex_code, wait=False, delay=60)
                sleep(500)  # Debounce

            sleep(50)

    def run(self):
        """Main control loop"""
        print("\nRemote Control Active")
        print("Waiting for IR commands...")
        print("-" * 40)

        last_command = None
        command_count = 0

        while True:
            # Check buttons
            if button_a.was_pressed():
                self.learn_mode = True
                self.learn_ir_codes()

            if button_b.was_pressed():
                print("Emergency stop!")
                self.robot.stop()
                display.show(Image.NO)
                sleep(1000)
                display.show(Image.HAPPY)

            # Read IR
            code = self.read_ir()

            if code is not None:
                # Look up command
                command = IR_COMMANDS.get(code)

                if command:
                    # Execute command
                    if command == last_command:
                        command_count += 1
                    else:
                        command_count = 1
                        last_command = command

                    self.execute_command(command)

                else:
                    # Unknown code
                    hex_code = "0x{:02X}".format(code)
                    print(f"Unknown IR code: {hex_code}")
                    print("  Run learn mode (button A) to identify codes")

                sleep(200)  # Debounce

            else:
                # No IR signal - stop if moving
                if last_command in ('forward', 'reverse', 'turn_left', 'turn_right'):
                    # Auto-stop after no signal
                    command_count += 1
                    if command_count > 5:  # ~1 second
                        self.robot.stop()
                        last_command = None
                        command_count = 0

            sleep(100)


def main():
    """Main program"""
    try:
        controller = RemoteControl()
        controller.run()

    except KeyboardInterrupt:
        print("\nStopped by user")

    except Exception as e:
        print(f"Error: {e}")
        display.show(Image.SAD)

    finally:
        # Cleanup
        try:
            robot = MaqueenPlusV3(laser_enabled=False)
            robot.stop()
        except:
            pass


if __name__ == "__main__":
    main()
