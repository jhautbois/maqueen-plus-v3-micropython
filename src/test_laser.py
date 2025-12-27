"""Test script for SEN0628 Laser Matrix Sensor - Simple version
Compatible with all MicroPython versions (no f-strings, no rjust)
"""
from microbit import display, sleep, Image, i2c, running_time
from laser_matrix import LaserMatrix

def pad(s, width):
    """Pad string to width with spaces"""
    s = str(s)
    while len(s) < width:
        s = " " + s
    return s


def main():
    """Main test sequence"""
    print("\n" + "="*50)
    print("LIDAR Test - Address 0x33")
    print("="*50)

    display.show(Image.HEART)
    sleep(500)

    # Step 1: Scan I2C bus
    print("\nStep 1: Scanning I2C bus...")
    devices = i2c.scan()
    print("Found " + str(len(devices)) + " device(s):")
    for addr in devices:
        print("  - " + str(addr) + " (0x" + hex(addr)[2:] + ")")

    # Step 2: Initialize sensor at 0x33
    print("\nStep 2: Init sensor at 0x33...")
    try:
        sensor = LaserMatrix(0x33)
        print("  OK - Sensor found!")
        display.show(Image.YES)
    except Exception as e:
        print("  ERROR: " + str(e))
        display.show(Image.SAD)
        return

    sleep(1000)

    # Step 3: Set 8x8 mode
    print("\nStep 3: Setting 8x8 mode...")
    if sensor.set_mode(LaserMatrix.MODE_8x8):
        print("  OK - Mode set")
    else:
        print("  Warning: Mode set failed")

    sleep(500)

    # Step 4: Test single point
    print("\nStep 4: Test center point (3,3)...")
    for i in range(5):
        dist = sensor.read_point(3, 3)
        if dist > 0:
            print("  " + str(i+1) + ". Distance: " + str(dist) + " mm")
        else:
            print("  " + str(i+1) + ". Read error")
        sleep(200)

    # Step 5: Test full matrix
    print("\nStep 5: Test 8x8 matrix...")
    matrix = sensor.read_matrix()

    if matrix:
        print("  OK - Got " + str(len(matrix)) + " readings")
        print("  Min: " + str(min(matrix)) + " mm")
        print("  Max: " + str(max(matrix)) + " mm")
        avg = sum(matrix) // len(matrix)
        print("  Avg: " + str(avg) + " mm")

        # Display matrix
        print("\n  8x8 Distance grid (mm):")
        for row in range(8):
            line = "    "
            for col in range(8):
                idx = row * 8 + col
                line += pad(matrix[idx], 4) + " "
            print(line)
    else:
        print("  ERROR - Failed to read matrix")
        display.show(Image.SAD)
        return

    # Step 6: Test zones
    print("\nStep 6: Test zones (L/C/R)...")
    for i in range(10):
        zones = sensor.read_zones()
        if zones:
            l = zones['left']
            c = zones['center']
            r = zones['right']
            msg = "  " + str(i+1) + ". L:" + pad(l, 4)
            msg += " C:" + pad(c, 4)
            msg += " R:" + pad(r, 4) + " mm"
            print(msg)
        else:
            print("  " + str(i+1) + ". Read error")
        sleep(200)

    # Step 7: Performance test
    print("\nStep 7: Performance test (30 samples)...")
    success = 0
    errors = 0
    times = []

    for i in range(30):
        start = running_time()
        zones = sensor.read_zones()
        elapsed = running_time() - start
        times.append(elapsed)

        if zones:
            success += 1
            c = zones['center']
            msg = "  " + pad(i+1, 2) + ". Center: "
            msg += pad(c, 4) + " mm (" + str(elapsed) + " ms)"
            print(msg)
        else:
            errors += 1
            print("  " + pad(i+1, 2) + ". ERROR")

        sleep(max(0, 66 - elapsed))

    # Statistics
    avg_time = sum(times) // len(times)
    if avg_time > 0:
        actual_hz = 1000 // avg_time
    else:
        actual_hz = 0

    print("\n  Success: " + str(success) + "/30")
    print("  Errors:  " + str(errors) + "/30")
    print("  Avg time: " + str(avg_time) + " ms")
    print("  Rate: " + str(actual_hz) + " Hz")

    # Final summary
    print("\n" + "="*50)
    print("Test Complete!")
    print("="*50)

    if success >= 25 and avg_time < 150:
        print("\nOK - Sensor works perfectly!")
        print("  Success rate: " + str(success*100//30) + "%")
        print("  Frequency: " + str(actual_hz) + " Hz")
        display.show(Image.HAPPY)
    elif success >= 15:
        print("\nWARNING - Sensor has issues")
        print("  Success: " + str(success*100//30) + "%")
        display.show(Image.MEH)
    else:
        print("\nERROR - Sensor has problems!")
        print("  Only " + str(success*100//30) + "% success")
        display.show(Image.SAD)

    print("\nPress RESET to run again")
    print("="*50 + "\n")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("\nFATAL ERROR: " + str(e))
        display.show(Image.SKULL)
        import sys
        sys.print_exception(e)
