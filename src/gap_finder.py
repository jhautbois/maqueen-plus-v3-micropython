"""Minimal Gap Finder - Anti-oscillation simple"""
from micropython import const

_CENTER_BONUS = const(115)
_STABILITY = const(2)


class GapFinder:
    __slots__ = ('last', 'cnt')

    def __init__(self):
        self.last = 4
        self.cnt = 0

    def find_gap(self, cols):
        if len(cols) != 8:
            return (0, 0, 0)

        # Trouver meilleur secteur avec bonus centre
        best_i = 4
        best_s = 0
        for i, d in enumerate(cols):
            s = d * _CENTER_BONUS // 100 if i == 3 or i == 4 else d
            if s > best_s:
                best_s = s
                best_i = i

        # Hysteresis simple
        if best_i != self.last:
            self.cnt += 1
            if self.cnt < _STABILITY:
                best_i = self.last
            else:
                self.cnt = 0
        else:
            self.cnt = 0

        self.last = best_i

        # Angle: centre=0, bords=+/-30
        if best_i == 3 or best_i == 4:
            angle = 0
        else:
            angle = int((best_i - 3.5) * 8.5)

        return (0, cols[best_i], angle)

    def reset(self):
        self.cnt = 0
