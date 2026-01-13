"""Grille d'occupation compacte 24x24"""
from micropython import const

_SIZE = const(24)
_CELL_MM = const(150)
_UNKNOWN = const(0)
_FREE = const(1)
_OCCUPIED = const(2)

# sin*256, cos*256 pour 8 colonnes LIDAR (-26 a +26 deg)
_SIN = (-113, -82, -50, -17, 17, 50, 82, 113)
_COS = (229, 243, 251, 256, 256, 251, 243, 229)

# Rotation heading: sin*256, cos*256 pour 8 directions
_H_SIN = (0, 181, 256, 181, 0, -181, -256, -181)
_H_COS = (256, 181, 0, -181, -256, -181, 0, 181)


class MiniGrid:
    def __init__(self):
        self._data = bytearray(144)  # 24x24 cells * 2 bits = 144 bytes
        self._ox = _SIZE // 2
        self._oy = _SIZE // 2

    def get(self, x, y):
        if x < 0 or x >= _SIZE or y < 0 or y >= _SIZE:
            return _UNKNOWN
        cell = y * _SIZE + x
        bi = cell >> 2
        bo = (cell & 3) << 1
        return (self._data[bi] >> bo) & 3

    def set(self, x, y, val):
        if x < 0 or x >= _SIZE or y < 0 or y >= _SIZE:
            return
        cell = y * _SIZE + x
        bi = cell >> 2
        bo = (cell & 3) << 1
        self._data[bi] = (self._data[bi] & ~(3 << bo)) | ((val & 3) << bo)

    def world_to_grid(self, x_mm, y_mm):
        gx = self._ox + x_mm // _CELL_MM
        gy = self._oy + y_mm // _CELL_MM
        return (gx, gy)

    def update_from_lidar(self, cols, robot_x_mm, robot_y_mm, heading_idx):
        rx, ry = self.world_to_grid(robot_x_mm, robot_y_mm)

        # Rotation selon heading
        h_sin = _H_SIN[heading_idx]
        h_cos = _H_COS[heading_idx]

        for i, d in enumerate(cols):
            if d <= 50 or d >= 2500:
                continue

            cells = d // _CELL_MM
            if cells < 1 or cells > 12:
                continue

            # Direction colonne LIDAR
            col_sin = _SIN[i]
            col_cos = _COS[i]

            # Rotation par heading: (sin, cos) * rotation_matrix
            # new_sin = col_sin * h_cos + col_cos * h_sin
            # new_cos = col_cos * h_cos - col_sin * h_sin
            rot_sin = (col_sin * h_cos + col_cos * h_sin) // 256
            rot_cos = (col_cos * h_cos - col_sin * h_sin) // 256

            # Position obstacle
            ox = rx + (cells * rot_sin) // 256
            oy = ry + (cells * rot_cos) // 256
            self.set(ox, oy, _OCCUPIED)

    def is_blocked(self, rx, ry, dx, dy):
        return self.get(rx + dx, ry + dy) == _OCCUPIED

    def count_occupied(self):
        count = 0
        for b in self._data:
            for i in range(4):
                if ((b >> (i * 2)) & 3) == _OCCUPIED:
                    count += 1
        return count

    def clear(self):
        for i in range(144):
            self._data[i] = 0

    def get_stats(self):
        occ = 0
        free = 0
        for b in self._data:
            for i in range(4):
                v = (b >> (i * 2)) & 3
                if v == _OCCUPIED:
                    occ += 1
                elif v == _FREE:
                    free += 1
        return (occ, free)
