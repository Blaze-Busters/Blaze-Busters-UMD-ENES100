from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
from enes100 import enes100
import time
from math import pi

servo = PWM(Pin(16), freq=50)

FS1 = Pin(17, Pin.IN)
FS2 = Pin(25, Pin.IN)
FS3 = Pin(26, Pin.IN)
FS4 = Pin(39, Pin.IN)

def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame,
                             *, active_low=True, stable=False, samples=5, delay_ms=2):

    sensors = [left_flame, right_flame, front_flame, back_flame]

    def read_value(sensor):
        if not stable:
            return sensor.value()
        from time import sleep_ms
        ones = 0
        for _ in range(samples):
            ones += sensor.value()
            sleep_ms(delay_ms)
        return 1 if ones >= (samples // 2 + 1) else 0

    expected = 0 if active_low else 1
    fire_count = sum(read_value(s) == expected for s in sensors)
    return fire_count + 1

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


print(number_of_flames_lit(FS1, FS2, FS3, FS4, active_low=True, stable=True, samples=10, delay_ms=5))
time.sleep(1)
spin(3, -50)
time.sleep(2)
spin(3, 50)
