#The Final Code Uploaded To The ESP32 To Run The Mission
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
enes100.begin("BlazeBusters", "FIRE", 67, 1120)
time.sleep(1)
enes100.is_connected()

# ----------------- SERVO FUNCTION -----------------
def spin(duration, speed):
    # speed expected in range -100 .. 100
    speed = max(min(speed, 100), -100)    # clamp
    
    min_us = 1000
    max_us = 2000
    center_us = 1500
    period_us = 20000    # 50Hz → 20ms period

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
    right_sensor_down = distance_cm(TRIG4, ECHO4)
    left_sensor_down = distance_cm(TRIG5, ECHO5)

# ----------------- MOTOR HELPERS -----------------
# Set PWM frequency
for en in (ENA, ENB):
    en.freq(5000)

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

# ----- motors_spin WITHOUT KICK -----
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


# ----- Continuous motor control (ON/OFF) WITHOUT KICK -----
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
 motors_spin(-,-) # straight
 motors_spin(+,+) # back
 motors_spin(+,-) # right
 motors_spin(-,+) # left
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
 #B, D
    option_A = (4 <= left_distance <= 5.5) and (5.2 <= right_distance <= 7.4)
    option_B = (1.7 <= left_distance <= 3.5) and (2.9 <= right_distance <= 4)

    option_C = (3.5 <= left_distance <= 4.3) and (3.4 <= right_distance <= 4.1)
    option_D = (5.1 <= left_distance <= 6.4) and (1.8 <= right_distance <= 3.4)

    if option_A:
        enes100.mission('TOPOGRAPHY', 'TOP_A')
        return "Option A"
    if option_B:
        enes100.mission('TOPOGRAPHY', 'TOP_B')
        return "Option B"
    if option_C:
        enes100.mission('TOPOGRAPHY', 'TOP_C')
        return "Option C"
    if option_D: return "Option D"
    return "Unknown — values do not match any option."

#-------FINAL TASK CODE-----

enes100.x
enes100.y
enes100.theta

while enes100.y == -1.00:
    time.sleep(0.1)
if enes100.y<=1:
    enes100.print(enes100.y)
    while not (1.15 < enes100.theta < 1.55):
        enes100.theta
        motor_on(-75, 70)  # spins until facing box
        time.sleep(.001)
    time.sleep(0.4)
if(enes100.y>1):
    enes100.print(enes100.y)
    while not (-2 < enes100.theta < -1.8):
        enes100.theta
        motor_on(-75, 70)  # spins until facing box
        time.sleep(.001)
    time.sleep(0.4)

# gets us into box
update_sensors()
while (front_sensor > 5):  # moves into box
    update_sensors()
    motor_on(88, 100)
    time.sleep(.05)
motor_off()
time.sleep(0.2)
motors_spin(2, 78, 100)  # snugs into box
time.sleep(1)

# detects how many candles are lit
update_sensors()
numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4, stable=True)
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")
enes100.print(f"Candles Lit: {fire_emoji}")
#enes100.mission('NUM_CANDLES', numberLit)

time.sleep(0.5)

# checks orientation of box
hold=0
while hold!=3:
    update_sensors()
    print(classify_position(left_sensor_down, right_sensor_down))
    enes100.print(classify_position(left_sensor_down, right_sensor_down))
    time.sleep(1)
    hold+=1
time.sleep(0.2)

# puts out candles
spin(2, -50)
time.sleep(2)
spin(2, 50)

time.sleep(0.4)

# back outs and turns
motors_spin(7, -88, -100)
time.sleep(0.4)

if(0<enes100.theta<2):
    motors_spin(1.85,88,-100)
else:
    motors_spin(1.85,-88,100)

#-----------ZONE 2-----------
update_sensors()
enes100.x
enes100.y
enes100.theta
while (enes100.x < 3):  # while still in obstacle range
    update_sensors()
    enes100.x
    # Gets us in front of an obstacle, stops before it
    front_sensor=50
    while front_sensor > 35:
        motor_on(87, 100)
        time.sleep(0.1)
        update_sensors()
    time.sleep(0.5)
    motor_off()
    time.sleep(0.5)
    update_sensors()

    # figuring out which way to turn
    if enes100.y < 0.8:  # if on right side of field
        enes100.y
        motors_spin(1.85,-88,100)  # turn left
        update_sensors()
        right_sensor_side=30
        while right_sensor_side < 40:  # move until past obstacle
            motor_on(87, 100)
            time.sleep(0.05)
            update_sensors()
        motor_off()
        time.sleep(0.2)
        motors_spin(0.7,87,100)
        time.sleep(1)
        motors_spin(1.85,88,-100)  # turn right
        time.sleep(0.5)

    else:
        motors_spin(1.85,88,-100)  # turn right
        update_sensors()
        left_sensor_side=30
        while left_sensor_side < 40:
            motor_on(87, 100)
            time.sleep(0.05)
            update_sensors()
        motor_off()
        time.sleep(0.2)
        motors_spin(0.7,87,100)
        time.sleep(1)
        motors_spin(1.85, -88, 100) # turn leftt
        time.sleep(0.5)

enes100.x
while enes100.x<3.85:
    motor_on(87,100)
    time.sleep(0.3)
time.sleep(0.1)
motor_off()











