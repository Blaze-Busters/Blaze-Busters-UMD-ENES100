from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
from enes100 import enes100
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

#MOTOR DRIVER SETUP
ENA = Pin(22, Pin.OUT)
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENB = Pin(23, Pin.OUT)
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4, Pin.OUT)
#SERVO PIN
servo = PWM(Pin(16), freq=50)

#SERVO FUNCTION
def spin(duration, speed):
    STOP_DUTY = 77
    MAX_RANGE = 25
    #speed
    speed = max(-100, min(100, speed))
    #convert speed (-100..100) → duty (52..102)
    duty = int(STOP_DUTY + (speed / 100) * MAX_RANGE)
    servo.duty(duty)
    #spin for desired duration
    time.sleep(duration)
    # Stop servo
    servo.duty(STOP_DUTY)

'''
GUIDE: SERVO
spin(2,50) #spin forward
spin(5,-100) #spin backward
'''

#ULTRASOUND
def distance_cm():
    TRIG.value(0)
    sleep_us(2)
    TRIG.value(1)
    sleep_us(10)
    TRIG.value(0)
    
    duration = time_pulse_us(ECHO, 1, 30000)  # 30ms timeout
    
    dist_cm = (duration / 2) * 0.0343
    return dist_cm
