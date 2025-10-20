from machine import Pin, time_pulse_us
from time import sleep_us, sleep
import time

TRIG = Pin(5, Pin.OUT)
ECHO = Pin(18, Pin.IN)
flame = Pin(34, Pin.IN)

def distance_cm():
    TRIG.value(0)
    sleep_us(2)
    TRIG.value(1)
    sleep_us(10)
    TRIG.value(0)
    
    duration = time_pulse_us(ECHO, 1, 30000)  # 30ms timeout
    
    dist_cm = (duration / 2) * 0.0343
    return dist_cm


def flame_detected():
    return flame.value() == 0

# enes100.begin("Blaze Busters","FIRE",222,1120)

# while (enes100.is_visible):
#     print(enes100.x)
#     print(enes100.y)
#     print(enes100.theta)
#     print("-----------------------------")
#     time.sleep(2)

while True:
    distance = distance_cm()
    print("Distance: {:.2f} cmm Flame Detected: {}".format(distance, flame_detected()))
    time.sleep(.5)