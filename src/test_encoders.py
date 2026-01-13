"""Test des registres encodeurs Maqueen Plus V3"""
from microbit import i2c, display, Image, button_a, sleep

display.show(Image.ARROW_N)
print("Test encodeurs - pousse le robot")
print("Registres: 0x04 (L), 0x06 (R)")

# Tester plusieurs registres possibles
REGS = [0x04, 0x06, 0x08, 0x0A, 0x3D, 0x3E, 0x4C, 0x4D]

while True:
    if button_a.was_pressed():
        break

    print("---")
    for reg in REGS:
        try:
            i2c.write(0x10, bytes([reg]))
            d = i2c.read(0x10, 2)
            val = d[0] | (d[1] << 8)
            if val != 0:
                print("0x" + hex(reg)[2:].upper() + ": " + str(val))
        except:
            pass

    sleep(500)

print("Fin")
display.show(Image.YES)
