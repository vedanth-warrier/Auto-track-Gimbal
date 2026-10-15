# from machine import Pin, I2S
# import array

# SCK, WS, SD = 5, 6, 3
# RATE = 16000
# BUF_BYTES = 4096

# mic = I2S(
#     0,
#     sck=Pin(SCK),
#     ws=Pin(WS),
#     sd=Pin(SD),
#     mode=I2S.RX,
#     bits=32,
#     format=I2S.STEREO,
#     rate=RATE,
#     ibuf=20000,
# )

# buf = bytearray(BUF_BYTES)

# while True:
#     n = mic.readinto(buf)
#     samples = array.array("i", buf[:n])
#     # INMP441 data is 24-bit left-justified in a 32-bit word
#     peak = max(abs(s >> 8) for s in samples)
#     print(peak)

from machine import Pin, I2S
import sys

RATE = 16000
BLK = 2048   # bytes per bus per packet (256 stereo frames)

a = I2S(0, sck=Pin(5), ws=Pin(6), sd=Pin(3), mode=I2S.RX,
        bits=32, format=I2S.STEREO, rate=RATE, ibuf=32000)
b = I2S(1, sck=Pin(8), ws=Pin(7), sd=Pin(4), mode=I2S.RX,
        bits=32, format=I2S.STEREO, rate=RATE, ibuf=32000)

bufa = bytearray(BLK)
bufb = bytearray(BLK)
out = sys.stdout.buffer

while True:
    a.readinto(bufa)
    b.readinto(bufb)
    out.write(b"SYNC")
    out.write(bufa)
    out.write(bufb)