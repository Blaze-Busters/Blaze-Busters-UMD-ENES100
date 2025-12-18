from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
from enes100 import enes100
import time
from math import pi

servo = PWM(Pin(16), freq=50)

def spin(duration, speed):
    # speed expected in range -100 .. 100
    speed = max(min(speed, 100), -100)    # clamp
    
    min_us = 1000
    max_us = 2000
    center_us = 1500
    period_us = 20000    # 50Hz → 20ms period

    # map speed (-100..100) to pulse (1000..2000)
    pulse = center_us + (speed / 100) * 500
    pulse = max(min(pulse, max_us), min_us)

    duty = int((pulse / period_us) * 65535)
    servo.duty_u16(duty)

    time.sleep(duration)

    # return to center
    neutral_duty = int((center_us / period_us) * 65535)
    servo.duty_u16(neutral_duty)

spin(3, -50)
time.sleep(2)
spin(3, 50)
