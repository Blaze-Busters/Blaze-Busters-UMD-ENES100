from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
from enes100 import enes100
import time
from math import pi

# ----------------- ULTRASONIC SENSOR PINS -----------------
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

# ----------------- FLAME SENSOR PINS -----------------
FS1 = Pin(17, Pin.IN)
FS2 = Pin(25, Pin.IN)
FS3 = Pin(26, Pin.IN)
FS4 = Pin(39, Pin.IN)

# ----------------- MOTOR DRIVER SETUP -----------------
ENA = PWM(Pin(22), freq=1000)
IN1 = Pin(18, Pin.OUT)
IN2 = Pin(19, Pin.OUT)
ENB = PWM(Pin(23), freq=1000)
IN3 = Pin(21, Pin.OUT)
IN4 = Pin(4, Pin.OUT)

# ----------------- SERVO PIN -----------------
servo = PWM(Pin(16), freq=50)

# ----------------- START BUTTON -----------------
button_start = Pin(15, Pin.IN, Pin.PULL_UP)

# ----------------- SERVO FUNCTION -----------------
def spin(duration, speed):
    # speed expected in range -100 .. 100
    speed = max(min(speed, 100), -100)   # clamp
    
    min_us = 1000
    max_us = 2000
    center_us = 1500
    period_us = 20000   # 50Hz → 20ms period

    # map speed (-100..100) to pulse (1000..2000)
    pulse = center_us + (speed / 100) * 500
    pulse = max(min(pulse, max_us), min_us)

    duty = int((pulse / period_us) * 65535)
    servo.duty_u16(duty)

    time.sleep(duration)

    # return to center
    neutral_duty = int((center_us / period_us) * 65535)
    servo.duty_u16(neutral_duty)


# ----------------- ULTRASOUND (CLEAN VERSION) -----------------
def distance_cm(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 30000)
    if duration <= 0:
        return 999  # No echo detected

    return (duration / 2) * 0.0343

# sensor readings (globals)
front_sensor = 50
left_sensor_side = 50
right_sensor_side = 50
left_sensor_down = 0
right_sensor_down = 0

def update_sensors():
    global front_sensor, left_sensor_side, right_sensor_side, left_sensor_down, right_sensor_down
    front_sensor = distance_cm(TRIG1, ECHO1)
    left_sensor_side = distance_cm(TRIG2, ECHO2)
    right_sensor_side = distance_cm(TRIG3, ECHO3)
    left_sensor_down = distance_cm(TRIG4, ECHO4)
    right_sensor_down = distance_cm(TRIG5, ECHO5)

# ----------------- MOTOR HELPERS -----------------
# Set PWM frequency
for en in (ENA, ENB):
    en.freq(1000)

# Make everything safe
for p in (IN1, IN2, IN3, IN4):
    p.value(0)

# ----- PWM helper -----
def _set_pwm(pwm, frac):
    if frac < 0:  frac = 0.0
    if frac > 1:  frac = 1.0
    try:
        pwm.duty_u16(int(frac * 65535))
    except AttributeError:
        pwm.duty(int(frac * 1023))


# ----- DC Motor class -----
class DCMotor:
    def __init__(self, in_a: Pin, in_b: Pin, en_pwm: PWM,
                 brake_stop=False, invert=False, gain=1.0):
        self.in_a = in_a
        self.in_b = in_b
        self.en   = en_pwm
        self.brake = brake_stop
        self.inv   = invert
        self.gain  = gain
        self.stop()

    def _dir(self, forward=True):
        fwd = forward ^ self.inv
        if fwd:
            self.in_a.value(1); self.in_b.value(0)
        else:
            self.in_a.value(0); self.in_b.value(1)

    def prepare_start(self, speed):
        if speed is None:
            speed = 0

        speed *= self.gain
        speed = max(min(speed, 100), -100)

        if speed == 0:
            return 0

        self._dir(forward=(speed > 0))
        return speed

    def apply_pwm(self, speed):
        if speed == 0:
            self.stop()
            return
        _set_pwm(self.en, abs(speed) / 100.0)

    def stop(self):
        if self.brake:
            self.in_a.value(1); self.in_b.value(1)
        else:
            self.in_a.value(0); self.in_b.value(0)
        _set_pwm(self.en, 0.0)


# ----- CREATE MOTORS -----
motor_left  = DCMotor(IN1, IN2, ENA, brake_stop=False, gain=1)
motor_right = DCMotor(IN3, IN4, ENB, brake_stop=False, gain=1)


#-----MOTOR SPIN FUNCTION -----
def motors_spin(duration, speed_left, speed_right):
    sL = motor_left.prepare_start(speed_left)
    sR = motor_right.prepare_start(speed_right)

    if sL == 0 and sR == 0:
        time.sleep(duration)
        return

    # Apply steady PWM immediately
    motor_left.apply_pwm(sL)
    motor_right.apply_pwm(sR)

    time.sleep(duration)

    motor_left.stop()
    motor_right.stop()


# ----- Continuous motor control -----
def motor_on(speed_left, speed_right):
    sL = motor_left.prepare_start(speed_left)
    sR = motor_right.prepare_start(speed_right)

    if sL == 0 and sR == 0:
        return

    motor_left.apply_pwm(sL)
    motor_right.apply_pwm(sR)


def motor_off():
    motor_left.stop()
    motor_right.stop()

'''
Directions:
 motors_spin(-,+) # straight
 motors_spin(+,-) # back
 motors_spin(+,+) # right
 motors_spin(-,-) # left
'''


# ----------------- CANDLE CHECK FUNCTION -----------------
def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame,
                         *, active_low=True, stable=False, samples=5, delay_ms=2):

    sensors = [left_flame, right_flame, front_flame, back_flame]

    def read_value(sensor):
        if not stable:
            return sensor.value()
        from time import sleep_ms
        ones = 0
        for _ in range(samples):
            ones += sensor.value()
            sleep_ms(delay_ms)
        return 1 if ones >= (samples // 2 + 1) else 0

    expected = 0 if active_low else 1
    fire_count = sum(read_value(s) == expected for s in sensors)
    return fire_count + 1
'''
numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4, stable=True)
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")
'''

# ----------------- ORIENTATION (CLEAN VERSION, NO DUPLICATES) -----------------
def classify_position(left_distance, right_distance):
    option_A = (3.5 <= left_distance <= 5.5) and (8 <= right_distance <= 12)
    option_B = (2.7 <= left_distance <= 5.2) and (4 <= right_distance <= 6.3)
    option_C = (4.3 <= left_distance <= 7) and (3.7 <= right_distance <= 4.9)
    option_D = (7 <= left_distance <= 30) and (2 <= right_distance <= 4)

    if option_A: return "Option A"
    if option_B: return "Option B"
    if option_C: return "Option C"
    if option_D: return "Option D"
    return "Unknown — values do not match any option."
#print(classify_position(left_sensor_down, right_sensor_down))

# ----------------- VICTORY DANCE -----------------
def victory_dance():
    motors_spin(0.5, 100, 100)
    motors_spin(0.3, -50, 100)
    motors_spin(0.3, -100, -100)
    motors_spin(0.3, -50, 100)
    motors_spin(0.3, 50, -100)

# ----------------- NAVIGATION RUN -----------------

# odometry / position globals
x = 0.0
y = 0.0
theta = 0.0

def update_position():
    """Single update of x, y, theta (no threading)."""
    global x, y, theta
    x = enes100.x()
    y = enes100.y()
    theta = enes100.theta()

# ----ZONE BOUNDARIES----
Z1_MAX = 0.76 #meters
Z2_MAX = 2.75
Z3_MAX = 3.7

# ----- state machine definitions -----
IDLE, ZONE1, ZONE2, ZONE3, DONE = range(5)
state = IDLE

'''
while True:
    time.sleep_ms(10)  # small loop delay
    update_position()
    update_sensors()

    # ---------- IDLE ----------
    if state == IDLE:
        motor_off()
        if button_start.value() == 0:   # button pressed (active low)
            time.sleep(0.2)            # simple debounce
            state = ZONE1

    # ---------- ZONE 1 ----------
    elif state == ZONE1:
        while x < Z1_MAX:
            update_position()
            update_sensors()

            if y < 1.3:
                # face +pi/2 (roughly straight up)
                while not (1.541 < theta < 1.599):
                    update_position()
                    motor_on(50, -50)      # tiny spin left
                    time.sleep(0.05)
                    motor_off()

                # move forward a bit
                motor_on(60, 60)
                time.sleep(2.5)
                motor_off()

            else:
                # face -pi/2 (roughly straight down)
                while not (-1.599 < theta < -1.541):
                    update_position()
                    motor_on(-50, 50)      # tiny spin right
                    time.sleep(0.05)
                    motor_off()

                # move forward a bit
                motor_on(60, 60)
                time.sleep(2.5)
                motor_off()

            # extinguish flames here (placeholder)
            if number_of_flames_lit(FS1, FS2, FS3, FS4, stable=True) > 1:
                spin(10, 100)   # lower
                spin(10,-100)   # raise

            motors_spin(4, 60, -60)  # backup

            # turn to face finish line (rough heuristic)
            update_position()
            if theta > 1:
                motors_spin(0.5, 60, 60)   # rotate one way
            else:
                motors_spin(0.5, 60, -60)  # rotate other way

            update_position()

        state = ZONE2

    # ---------- ZONE 2 ----------
    elif state == ZONE2:
        while Z1_MAX < x < Z2_MAX:
            update_position()
            update_sensors()

            # read your ultrasonic sensors
            ultrasound_front = front_sensor
            ultrasound_right = right_sensor_side
            ultrasound_left = left_sensor_side

            # move straight until obstacle within 10 cm
            while ultrasound_front > 10 and Z1_MAX < x < Z2_MAX:
                update_position()
                update_sensors()
                motor_on(60, 60)
                time.sleep(0.05)
                ultrasound_front = front_sensor
            motor_off()

            if ultrasound_front < 10:
                if y > 1.5:
                    # on LEFT side of field → turn RIGHT
                    motors_spin(0.4, -60, 60)  # right 90°
                    while front_sensor < 15 and Z1_MAX < x < Z2_MAX:
                        update_position()
                        update_sensors()
                        motor_on(60, 60)
                        time.sleep(0.05)
                    motor_off()
                    motors_spin(0.4, 60, -60)  # left 90° back to lane

                elif y < 0.5:
                    # on RIGHT side of field → turn LEFT
                    motors_spin(0.4, 60, -60)  # left 90°
                    while front_sensor < 15 and Z1_MAX < x < Z2_MAX:
                        update_position()
                        update_sensors()
                        motor_on(60, 60)
                        time.sleep(0.05)
                    motor_off()
                    motors_spin(0.4, -60, 60)  # right 90° back to lane

                else:
                    # center → also turn LEFT
                    motors_spin(0.4, 60, -60)  # turn left
                    while front_sensor < 15 and Z1_MAX < x < Z2_MAX:
                        update_position()
                        update_sensors()
                        motor_on(60, 60)
                        time.sleep(0.05)
                    motor_off()
                    motors_spin(0.4, -60, 60)  # turn back right

        state = ZONE3

    # ---------- ZONE 3 ----------
    elif state == ZONE3:
        while Z2_MAX < x < Z3_MAX:
            update_position()
            update_sensors()

            if y < 1.5:
                # turn left 90
                motors_spin(0.4, 60, -60)
                while y < 1.5 and Z2_MAX < x < Z3_MAX:
                    update_position()
                    motor_on(60, 60)
                    time.sleep(0.05)
                motor_off()
            else:
                # move forward
                motor_on(60, 60)
                time.sleep(5)
                motor_off()

        state = DONE

    # ---------- DONE ----------
    elif state == DONE:
        motor_off()
        victory_dance()
        while True:
            time.sleep(1)  # WHAT ARE U GONNA GET? 100!
'''
