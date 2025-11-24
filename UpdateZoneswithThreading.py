import time
import _thread
from time import sleep_us, sleep
from enes100 import enes100
from machine import Pin, time_pulse_us, ADC, PWM
from NusCmandles import number_of_flames_lit
from MotorControlFunctions import DCMotor, motor_left, motor_right, motors_spin
#motors_spin(duration, speed_left, speed_right) main function used
from Orientation import classify_position

# ==== CONSTANTS ====
UPDATE_MS = 1000  # 1 second update rate
Z1_MAX = 0.80
Z2_MAX = 2.80
Z3_MAX = 3.80

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
# -----Pin map-----
# Left motor
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENA = PWM(Pin(22))         

# Right motor
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4,  Pin.OUT)
ENB = PWM(Pin(23))


button_start = Pin(16, Pin.IN, Pin.PULL_UP)

# ==== GLOBAL POSITION VARIABLES (auto-updated thread) ====
x = 0
y = 0
theta = 0

def update_pose():
    global x, y, theta
    x = enes100.x()
    y = enes100.y()
    theta = enes100.theta()

#MOTOR PWM SETUP
# PWM frequency
for en in (ENA, ENB):
    en.freq(1000)

# Make sure everything starts safe
for p in (IN1, IN2, IN3, IN4):
    p.value(0)

def _set_pwm(pwm, frac):  # frac: 0.0..1.0
    if frac < 0:  frac = 0.0
    if frac > 1:  frac = 1.0
    # Use duty_u16 for Pico (16-bit) or duty() for older/other MicroPython (10-bit)
    try:
        pwm.duty_u16(int(frac * 65535))
    except AttributeError:
        pwm.duty(int(frac * 1023))

# Distance function
def distance_cm(trig, echo):
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)
    dist_cm = (duration / 2) * 0.0343
    return dist_cm

front_sensor = 0
left_sensor_side = 0
right_sensor_side = 0
left_sensor_down = 0
right_sensor_down = 0

def update_sensors():
    global front_sensor, left_sensor_side, right_sensor_side, left_sensor_down, right_sensor_down
    while True:
        front_sensor = distance_cm(TRIG5, ECHO5)
        left_sensor_side = distance_cm(TRIG1, ECHO1)
        right_sensor_side = distance_cm(TRIG2, ECHO2)
        left_sensor_down = distance_cm(TRIG3, ECHO3)
        right_sensor_down = distance_cm(TRIG4, ECHO4)

        time.sleep(0.1)   # Update at 10Hz


# ==== STATE MACHINE DEFINITIONS ====
IDLE, ZONE1, ZONE2, ZONE3, DONE = range(5)
state = IDLE

while True:
    time.sleep_ms(10)  # small loop delay

    # ---------- IDLE ----------
    if state == IDLE:
        if button_start.value() == 0:
            state = ZONE1

    # ---------- ZONE 1 ----------
    elif state == ZONE1:
        while x < Z1_MAX:
            if y < 1.3:
                # face +pi/2 (1.541 to 1.599 rad)
                while not (1.541 < theta < 1.599):
                    motors_spin(1, 50, -50)
                    update_pose()
                    pass
                
                motors_spin(1, 50, -50)
                update_pose()
                #detect number of flames
                #extengiuish flames             
                pass

            else:
                # face -pi/2 (-1.599 to -1.541 rad)
                while not (-1.599 < theta < -1.541):
                    motors_spin(1, 50, -50)
                    update_pose()
                    pass

                motors_spin(7, 50, 50)
                update_pose()
                pass

            # extinguish flames here
            pass
        
        state = ZONE2

    # ---------- ZONE 2 ----------
    elif state == ZONE2:
        while Z1_MAX < x < Z2_MAX:

            # read your ultrasonic sensors somewhere
            # ultrasound_front = ???

            while ultrasound_front > 10:
                pass

            if ultrasound_front < 10:
                if y > 1.5:
                    # right 90, forward until clear, left 90
                    pass
                elif y > 0.8:
                    # same logic
                    pass
                else:
                    # left 90, forward until clear, right 90
                    pass
        
        state = ZONE3

    # ---------- ZONE 3 ----------
    elif state == ZONE3:
        while Z2_MAX < x < Z3_MAX:
            if y < 1.5:
                # turn left 90
                while y < 1.5:
                    # move forward
                    pass
            else:
                # move forward
                pass

        state = DONE

    # ---------- DONE ----------
    elif state == DONE:
        # stop motors, celebrate, etc.
        pass
