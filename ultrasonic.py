from machine import Pin, time_pulse_us
from time import sleep_us, sleep

TRIG = Pin(5, Pin.OUT)
ECHO = Pin(18, Pin.IN)

def distance_cm():
    TRIG.value(0)
    sleep_us(2)
    TRIG.value(1)
    sleep_us(10)
    TRIG.value(0)
    
    duration = time_pulse_us(ECHO, 1, 30000)  # 30ms timeout
    
    dist_cm = (duration / 2) * 0.0343
    return dist_cm
