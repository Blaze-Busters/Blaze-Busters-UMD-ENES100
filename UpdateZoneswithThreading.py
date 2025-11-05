import time
import threading
from enes100 import enes100
from machine import Pin, PWM
from NusCmandles import number_of_flames_lit
from MotorControlFunctions import DCMotor, motor_left, motor_right, motors_spin
#motors_spin(duration, speed_left, speed_right) main function used
from Orientation import classify_position
from main import distance_cm
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

def update_position():
    global x, y, theta
    while True:
        time.sleep(1)   # update every second
        x = enes100.x()
        y = enes100.y()
        theta = enes100.theta()

# start background pose updater
thread = threading.Thread(target=update_position, daemon=True)
thread.start()

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
                    # spin OTV to face correct direction
                    pass
                
                # move forward
                pass

            else:
                # face -pi/2 (-1.599 to -1.541 rad)
                while not (-1.599 < theta < -1.541):
                    # spin to correct direction
                    pass

                # move forward
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
                # move forward
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
