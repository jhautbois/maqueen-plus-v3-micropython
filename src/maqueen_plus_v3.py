"""Maqueen Plus V3 - Minimal"""
from microbit import i2c, pin1, sleep
import neopixel


class MaqueenPlusV3:
    HL = {'red': 1, 'green': 2, 'yellow': 3, 'blue': 4, 'purple': 6, 'white': 7}

    def __init__(self, laser_enabled=False):
        if 0x10 not in i2c.scan():
            raise RuntimeError("STM8 not found")
        self.neo = neopixel.NeoPixel(pin1, 4)
        self.underglow_off()

    def _wr(self, reg, data):
        if isinstance(data, int):
            data = bytes([data])
        i2c.write(0x10, bytes([reg]) + data)

    def motors(self, l, r):
        l = max(-255, min(255, l))
        r = max(-255, min(255, r))
        self._wr(0x00, bytes([1 if l < 0 else 0, abs(l)]))
        self._wr(0x02, bytes([1 if r < 0 else 0, abs(r)]))

    def drive(self, s):
        self.motors(s, s)

    def turn(self, s):
        self.motors(-s, s)

    def stop(self):
        self.motors(0, 0)

    def headlights(self, l, r):
        if isinstance(l, str):
            l = self.HL.get(l, 0)
        if isinstance(r, str):
            r = self.HL.get(r, 0)
        try:
            self._wr(0x0B, l)
            sleep(10)
            self._wr(0x0C, r)
        except:
            pass

    def underglow(self, idx, r, g, b):
        if idx == 'all':
            for i in range(4):
                self.neo[i] = (r, g, b)
        else:
            self.neo[idx] = (r, g, b)
        self.neo.show()

    def underglow_off(self):
        self.underglow('all', 0, 0, 0)
