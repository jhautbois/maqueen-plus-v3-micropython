"""MicroSLAM - Odometrie via vitesse + Grille 8x8"""
from microbit import i2c

_D = (0, 181, 128, 128, 181, 0, 128, -128, 0, -181, -128, -128, -181, 0, -128, 128)


class MicroSLAM:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.hd = 0
        self.hi = 0
        self.g = bytearray(16)

    def update(self, ml, mr):
        # ml, mr = motor commands (-255 to 255)
        # Estimate distance from motor commands (rough approximation)
        avg = (ml + mr) // 2
        diff = mr - ml
        # Update heading (turn rate proportional to motor difference)
        self.hd = (self.hd + diff // 8) % 360
        self.hi = self.hd // 45
        # Update position if moving
        if abs(avg) > 20:
            l = avg // 3  # Scale factor
            j = self.hi * 2
            self.x += l * _D[j] // 181
            self.y += l * _D[j + 1] // 181
        return avg

    def pose(self):
        return self.x, self.y, self.hd

    def lidar(self, c):
        rx = 4 + self.x // 200
        ry = 4 + self.y // 200
        hs = _D[self.hi * 2]
        hc = _D[self.hi * 2 + 1]
        for i, d in enumerate(c):
            if 50 < d < 1600:
                n = d // 200
                lx = (i - 4) * n // 8
                ox = rx + (lx * hc + n * hs) // 181
                oy = ry + (n * hc - lx * hs) // 181
                if 0 <= ox < 8 and 0 <= oy < 8:
                    k = oy * 8 + ox
                    self.g[k >> 2] |= 2 << ((k & 3) << 1)

    def reset(self):
        self.x = 0
        self.y = 0
        self.hd = 0
        self.hi = 0
        for i in range(16):
            self.g[i] = 0
