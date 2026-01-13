"""Minimal Fall Detection - Tilt et chute libre"""
from microbit import accelerometer
from micropython import const
import math

_TILT = const(800)
_FREEFALL = const(150)
_FF_FRAMES = const(2)


class FallDetector:
    __slots__ = ('ff_cnt',)

    def __init__(self, enable_cliff=False):
        self.ff_cnt = 0

    def check_fast(self):
        x, y, z = accelerometer.get_values()

        # Chute libre
        total = int(math.sqrt(x * x + y * y + z * z))
        if total < _FREEFALL:
            self.ff_cnt += 1
            if self.ff_cnt >= _FF_FRAMES:
                return (True, "freefall")
        else:
            self.ff_cnt = 0

        # Inclinaison (micro:bit vertical, Y~+1000 au repos)
        if abs(x) > _TILT:
            return (True, "tilt_side")
        if abs(z) > _TILT:
            return (True, "tilt_fwd")
        if y < 300:
            return (True, "pickup")

        return (False, None)
