from machine import Pin, time_pulse_us, ADC, PWM
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

def set_speed(speed):
    stop_duty = 77 
    duty_range = 25 
    duty = int(stop_duty + (speed / 100) * duty_range)
    servo.duty(duty)

def distance_cm(trig, echo):
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)
    dist_cm = (duration / 2) * 0.0343
    return dist_cm


def flame_detected(flame_pin):
    if flame_pin.value() == 0:
        return "Fire!"
    else:
        return "No Fire"

# enes100.begin("Blaze Busters","FIRE",222,1120)

# while (enes100.is_visible):
#     print(enes100.x)
#     print(enes100.y)
#     print(enes100.theta)
#     print("-----------------------------")
#     time.sleep(2)


while True:
    set_speed(pot.read() * 100 / 1023)
    print("""
ULTRASONIC SENSORS
US1: {} cm, US2: {} cm, US3: {} cm, US4: {} cm, US5: {} cm
FLAME SENSORS
FS1: {}, FS2: {}, FS3: {}, FS4: {}
POTENTIOMETER VALUE: {}
""".format(distance_cm(TRIG1, ECHO1),
           distance_cm(TRIG2, ECHO2),
           distance_cm(TRIG3, ECHO3),
           distance_cm(TRIG4, ECHO4),
           distance_cm(TRIG5, ECHO5),
           flame_detected(FS1),
           flame_detected(FS2),
           flame_detected(FS3),
           flame_detected(FS4))
    )
    time.sleep(.5)