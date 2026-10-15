from machine import Pin, PWM
from time import sleep

servo = PWM(Pin(12), freq=50)

def angle(deg, min_us=600, max_us=2400):
    deg = max(0, min(180, deg))
    us = min_us + (max_us - min_us) * deg / 180
    servo.duty_ns(int(us * 1000))

for a in (0, 90):
    angle(a)
    sleep(1)

servo.deinit()