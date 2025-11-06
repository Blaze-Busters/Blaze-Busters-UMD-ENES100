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
def distance_cm(): '''varun make sure this is right idk some pins changed its probs fine but im paranoid'''
    TRIG.value(0)
    sleep_us(2)
    TRIG.value(1)
    sleep_us(10)
    TRIG.value(0)
    
    duration = time_pulse_us(ECHO, 1, 30000)  # 30ms timeout
    
    dist_cm = (duration / 2) * 0.0343
    return dist_cm

#MOTOR FUNCTIONS
# PWM frequency
for en in (ENA, ENB):
    en.freq(1000)

# Make sure everything starts safe
for p in (IN1, IN2, IN3, IN4):
    p.value(0)

# ----- PWM helper -----
def _set_pwm(pwm, frac):  # frac: 0.0..1.0
    if frac < 0:  frac = 0.0
    if frac > 1:  frac = 1.0
    # Use duty_u16 for Pico (16-bit) or duty() for older/other MicroPython (10-bit)
    try:
        pwm.duty_u16(int(frac * 65535))
    except AttributeError:
        pwm.duty(int(frac * 1023))

# ----- Generic DC motor wrapper with the same API as your servo.spin -----
class DCMotor:
    def __init__(self, in_a: Pin, in_b: Pin, en_pwm: PWM, brake_stop=False, invert=False):
        self.in_a = in_a
        self.in_b = in_b
        self.en   = en_pwm
        self.brake = brake_stop    # False = coast stop; True = active brake
        self.inv   = invert        # flip direction if your wiring is reversed
        self.stop()                # start stopped

    def _dir(self, forward=True):
        fwd = forward ^ self.inv
        if fwd:
            self.in_a.value(1); self.in_b.value(0)
        else:
            self.in_a.value(0); self.in_b.value(1)

    def set(self, speed):
        # clamp and split into direction + magnitude
        if speed is None:
            speed = 0
        if speed > 100:  speed = 100
        if speed < -100: speed = -100

        if speed == 0:
            self.stop()
            return

        self._dir(forward=(speed > 0))
        mag = abs(speed) / 100.0
        _set_pwm(self.en, mag)

    def spin(self, duration, speed):
        self.set(speed)
        time.sleep(duration)
        self.stop()

    def stop(self):
        if self.brake:
            # active brake (both inputs high) for snappier stops
            self.in_a.value(1); self.in_b.value(1)
        else:
            # coast (both inputs low)
            self.in_a.value(0); self.in_b.value(0)
        _set_pwm(self.en, 0.0)

motor_left  = DCMotor(IN1, IN2, ENA, brake_stop=False)  # set brake_stop=True if you want sharper stops
motor_right = DCMotor(IN3, IN4, ENB, brake_stop=False)

def motor_left_spin(duration, speed):  motor_left.spin(duration, speed)
def motor_right_spin(duration, speed): motor_right.spin(duration, speed)

# spin both motors together
def motors_spin(duration, speed_left, speed_right):
    motor_left.set(speed_left)
    motor_right.set(speed_right)
    time.sleep(duration)
    motor_left.stop()
    motor_right.stop()
    
'''GUIDE: MOTOR
motors_spin(10, 80, 80)    #first 80 left motor
'''

#CANDLE CHECK FUNCTION

