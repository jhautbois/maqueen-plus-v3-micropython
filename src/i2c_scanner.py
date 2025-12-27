"""I2C Scanner for Maqueen Plus V3
Scans the I2C bus and displays all connected devices.
Flash this to micro:bit to identify component addresses.
"""
from microbit import display, i2c, sleep, Image

def scan_i2c():
    """Scan I2C bus and return list of addresses"""
    display.show(Image.HEART)
    devices = i2c.scan()
    return devices

def display_results(devices):
    """Display scan results on console and LED matrix"""
    if not devices:
        print("No I2C devices found!")
        display.show(Image.SAD)
    else:
        print(f"Found {len(devices)} I2C device(s):")
        for addr in devices:
            hex_addr = "0x{:02X}".format(addr)
            print(f"  - {hex_addr} ({addr})")

            # Identify known devices
            if addr == 0x10:
                print("    -> STM8 Motor Controller")
            elif addr in (0x30, 0x31, 0x32, 0x33):
                print("    -> Laser ToF Matrix Sensor")
            elif addr == 0x29:
                print("    -> Possible VL53L0X ToF sensor")
            else:
                print("    -> Unknown device")

        display.show(Image.HAPPY)

    return devices

def main():
    """Main function"""
    print("\n" + "="*40)
    print("Maqueen Plus V3 - I2C Scanner")
    print("="*40 + "\n")

    sleep(500)  # Wait for serial connection

    devices = scan_i2c()
    display_results(devices)

    print("\n" + "="*40)
    print("Scan complete!")
    print("="*40 + "\n")

if __name__ == "__main__":
    main()
