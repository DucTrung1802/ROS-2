# Author: skoehler
# Github: https://github.com/skoehler

# A very good solution to this can be found here.
# The author states:
# The code below gives me 790 kB/sec while replacing the code with pyserial's readline method gives me just 170kB/sec.
# There is no statement about the baud rate set for this comparison. The value of 9600 baud in the example below is only for testing.
# This solution also avoids having 100 % CPU usage.


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[: i + 1]
            self.buf = self.buf[i + 1 :]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[: i + 1]
                self.buf[0:] = data[i + 1 :]
                return r
            else:
                self.buf.extend(data)
