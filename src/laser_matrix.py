"""Laser ToF Matrix Sensor Driver (minified)"""
from microbit import i2c, sleep

class LaserMatrix:
    DEFAULT_ADDR = 0x33
    CMD_SETMODE = 0x01
    CMD_ALLDATA = 0x02
    CMD_FIXED_POINT = 0x03
    PACKET_HEAD = 0x55
    STATUS_SUCCESS = 0x53
    MODE_4x4 = 4
    MODE_8x8 = 8
    I2C_MAX = 32

    def __init__(self, addr=DEFAULT_ADDR):
        self.addr = addr
        self.mode = self.MODE_8x8
        if addr not in i2c.scan():
            raise RuntimeError("Sensor not found at " + hex(addr))
        i2c.write(addr, bytes([]))
        sleep(50)
        print("Laser OK at " + hex(addr))

    def _send(self, cmd, args=None):
        if args is None:
            args = []
        pkt = bytearray([self.PACKET_HEAD, (len(args) >> 8) & 0xFF, len(args) & 0xFF, cmd])
        for a in args:
            pkt.append(a)
        try:
            if len(pkt) <= self.I2C_MAX:
                i2c.write(self.addr, pkt)
            else:
                for i in range(0, len(pkt), self.I2C_MAX):
                    chunk = pkt[i:i + self.I2C_MAX]
                    i2c.write(self.addr, chunk)
                    sleep(10)
            return True
        except:
            return False

    def _recv(self, max_len=256):
        try:
            hdr = i2c.read(self.addr, 4)
            status, cmd = hdr[0], hdr[1]
            dlen = (hdr[3] << 8) | hdr[2]
            data = bytearray()
            if dlen > 0:
                rem = min(dlen, max_len)
                while rem > 0:
                    sz = min(rem, self.I2C_MAX)
                    chunk = i2c.read(self.addr, sz)
                    data.extend(chunk)
                    rem -= sz
                    if rem > 0:
                        sleep(5)
            return (status, cmd, bytes(data))
        except:
            return (None, None, None)

    def set_mode(self, mode):
        if mode not in (self.MODE_4x4, self.MODE_8x8):
            return False
        if not self._send(self.CMD_SETMODE, [mode]):
            return False
        sleep(100)
        status, cmd, data = self._recv()
        if status == self.STATUS_SUCCESS:
            self.mode = mode
            print("Waiting 5s for mode change...")
            sleep(5000)  # Critical: 5 second delay after mode change
            return True
        return False

    def read_matrix(self):
        # Read all points individually (CMD_ALLDATA doesn't work)
        dist = []
        for y in range(self.mode):
            for x in range(self.mode):
                d = self.read_point(x, y)
                if d < 0:
                    d = 4000
                dist.append(d)
        return dist

    def read_point(self, x, y):
        if x >= self.mode or y >= self.mode:
            return -1
        if not self._send(self.CMD_FIXED_POINT, [x, y]):
            return -1
        sleep(30)
        status, cmd, data = self._recv()
        if status == self.STATUS_SUCCESS and len(data) >= 2:
            return data[0] | (data[1] << 8)
        return -1

    def read_zones(self):
        m = self.read_matrix()
        if m is None:
            return None
        zw = self.mode // 3
        ls, cs, rs = 0, 0, 0
        lc, cc, rc = 0, 0, 0
        for r in range(self.mode):
            for c in range(self.mode):
                idx = r * self.mode + c
                if idx >= len(m):
                    break
                d = m[idx]
                if d >= 4000:
                    continue
                if c < zw:
                    ls += d
                    lc += 1
                elif c < 2 * zw:
                    cs += d
                    cc += 1
                else:
                    rs += d
                    rc += 1
        la = ls // lc if lc > 0 else 4000
        ca = cs // cc if cc > 0 else 4000
        ra = rs // rc if rc > 0 else 4000
        return {'left': la, 'center': ca, 'right': ra}
