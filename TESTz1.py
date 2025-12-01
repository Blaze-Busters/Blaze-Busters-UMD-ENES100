import time
from time import sleep_us, sleep
import _thread
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

x = 0
y = 0
theta = 0

def update_position():
    global x, y, theta
    while True:
        time.sleep(1)   # update every second
        x = enes100.x()
        y = enes100.y()
        theta = enes100.theta()


update_position()
while not (1.541 < theta < 1.599):
  # spin OTV to face correct direction
  motors_spin(0.1, -0.3, 0.3)
  update_position():
  pass

motors_spin(7, 0.3, 0.3)
 pass
