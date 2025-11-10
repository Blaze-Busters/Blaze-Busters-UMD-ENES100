import time
from time import sleep_us, sleep
from enes100 import enes100
from machine import Pin, time_pulse_us, ADC, PWM
from MotorControlFunctions import DCMotor, motor_left, motor_right, motors_spin

#MOTOR DRIVER SETUP
# -----Pin map-----
# Left motor
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENA = PWM(Pin(22))         

# Right motor
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4,  Pin.OUT)
ENB = PWM(Pin(23))

while not (1.541 < theta < 1.599):
  # spin OTV to face correct direction
  motors_spin(0.1, -0.3, 0.3)
    pass

motors_spin(0.1, 0.3, 0.3)
 pass
