from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
import time

servo = PWM(Pin(16), freq=50)

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

#calling it
spin(2,50) #spin forward
spin(5,-100) #spin backward
