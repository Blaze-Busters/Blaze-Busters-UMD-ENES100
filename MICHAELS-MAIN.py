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

#SERVO SETUP
servo = PWM(Pin(16), freq=50)
# pot = ADC(Pin(4))
# pot.atten(ADC.ATTN_11DB)
# pot.width(ADC.WIDTH_10BIT) 

#MOTOR DRIVER SETUP
ENA = Pin(22, Pin.OUT)
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENB = Pin(23, Pin.OUT)
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4, Pin.OUT)
