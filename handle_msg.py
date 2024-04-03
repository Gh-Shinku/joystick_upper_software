import zlib


class handleMsg(object):
    def __init__(self, source):
        self.head = 0x2B
        self.tail = 0x2A
        self.stamp = 0
        self.action = [0 for i in range(4)]

        self.button_array = [0 for i in range(16)]
        self.reserve = 0
        self.crc32 = 0
        self.source = ""

    def calc(self, msg, source):
        if msg[0] != self.head or msg[-1] != self.tail:
            return -1
        tmp_crc32 = int.from_bytes(msg[17:21], byteorder='little')
        checksum = zlib.crc32(bytearray(msg[0:17]))
        if tmp_crc32 != checksum:
            return -2
        self.source = source
        self.crc32 = checksum

        stamp = int.from_bytes(msg[1:5], byteorder="little", signed=True)
        if stamp == self.stamp:
            return -3
        self.stamp = stamp
        self.action = [int.from_bytes(msg[i:i + 2], byteorder="little", signed=True) for i in range(5, 13, 2)]
        num = int.from_bytes(msg[13:15], byteorder="little", signed=True)
        self.button_array = [(num >> bit) & 1 for bit in range(num.bit_length())]
        if len(self.button_array) != 16:
            self.button_array += ([0 for i in range(16 - len(self.button_array))])
        return 0

    def print(self):
        print("Source {} ".format(self.source), end="")
        print("Stamp {} ".format(self.stamp), end="")
        print("Action {} ".format(self.action), end="")
        print("Button {} ".format(self.button_array), end="")
        print("Reserve {}".format(self.reserve))
