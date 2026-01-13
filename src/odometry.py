"""Odometrie simple pour Maqueen Plus V3"""
from microbit import i2c
from micropython import const

_STM8 = const(0x10)
_REG_ENC_L = const(0x04)
_REG_ENC_R = const(0x06)
_TICKS_TO_MM = const(33)
_WHEEL_BASE = const(80)

# Directions 8 points cardinaux - (dx, dy) * 181
_DIR_X = (0, 128, 181, 128, 0, -128, -181, -128)
_DIR_Y = (181, 128, 0, -128, -181, -128, 0, 128)


class Odometry:
    def __init__(self):
        self._last_l = 0
        self._last_r = 0
        self._x = 0
        self._y = 0
        self._heading = 0
        self._heading_deg = 0
        self._init = False

    def _read_enc(self, reg):
        try:
            i2c.write(_STM8, bytes([reg]))
            d = i2c.read(_STM8, 2)
            return d[0] | (d[1] << 8)
        except:
            return 0

    def update(self):
        if not self._init:
            self._last_l = self._read_enc(_REG_ENC_L)
            self._last_r = self._read_enc(_REG_ENC_R)
            self._init = True
            return 0, 0

        enc_l = self._read_enc(_REG_ENC_L)
        enc_r = self._read_enc(_REG_ENC_R)

        dl = enc_l - self._last_l
        dr = enc_r - self._last_r
        if dl > 32768:
            dl -= 65536
        if dl < -32768:
            dl += 65536
        if dr > 32768:
            dr -= 65536
        if dr < -32768:
            dr += 65536

        self._last_l = enc_l
        self._last_r = enc_r

        mm_l = dl * _TICKS_TO_MM
        mm_r = dr * _TICKS_TO_MM

        linear = (mm_l + mm_r) // 2
        angular = (mm_r - mm_l) * 57 // _WHEEL_BASE

        self._heading_deg = (self._heading_deg + angular) % 360
        self._heading = self._heading_deg // 45

        if abs(linear) > 5:
            dx = linear * _DIR_X[self._heading] // 181
            dy = linear * _DIR_Y[self._heading] // 181
            self._x += dx
            self._y += dy

        return linear, angular

    def get_pose(self):
        return (self._x, self._y, self._heading_deg)

    def get_heading_idx(self):
        return self._heading

    def get_grid_pos(self, cell_mm):
        return (self._x // cell_mm, self._y // cell_mm)

    def reset(self):
        self._x = 0
        self._y = 0
        self._heading = 0
        self._heading_deg = 0
