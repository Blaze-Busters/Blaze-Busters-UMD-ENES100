from machine import Pin, time_pulse_us
from time import sleep_us, sleep
import time

#ULTRASONIC SENSOR PINS
TRIG1 = Pin(13, Pin.OUT)
ECHO1 = Pin(32, Pin.IN)
TRIG2 = Pin(12, Pin.OUT)
ECHO2 = Pin(33, Pin.IN)
TRIG3 = Pin(14, Pin.OUT)
ECHO3 = Pin(34, Pin.IN)
TRIG4 = Pin(27, Pin.OUT)
ECHO4 = Pin(35, Pin.IN)
TRIG5 = Pin(5, Pin.OUT)
ECHO5 = Pin(36, Pin.IN)

#FLAME SENSOR PINS
FS1 = Pin(17, Pin.IN)
FS2 = Pin(25, Pin.IN)
FS3 = Pin(26, Pin.IN)
FS4 = Pin(39, Pin.IN)


flame = Pin(34, Pin.IN)

def distance_cm(trig, echo):
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)
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
    print("""
ULTRASONIC SENSORS
US1: {} cm
US2: {} cm
US3: {} cm  
US4: {} cm
US5: {} cm
""".format(distance_cm(TRIG1, ECHO1),
           distance_cm(TRIG2, ECHO2),
           distance_cm(TRIG3, ECHO3),
           distance_cm(TRIG4, ECHO4),
           distance_cm(TRIG5, ECHO5))
    )
    time.sleep(.5)