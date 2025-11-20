from machine import Pin, time_pulse_us, ADC, PWM
from time import sleep_us, sleep
from enes100 import enes100
import _thread
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
#fix threading
def distance_cm(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    
    duration = machine.time_pulse_us(echo, 1, 30000)  # wait for echo HIGH max 30ms
    dist_cm = (duration / 2) * 0.0343
    return dist_cm

#sensor readings
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

_thread.start_new_thread(update_sensors, ())
'''GUIDE: distance=distance_cm(trig,echo)'''

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
def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame, *, active_low=True, stable=False, samples=5, delay_ms=2): #i asked gpt to add debouncing, resulted in more parameters

    sensors = [left_flame, right_flame, front_flame, back_flame]

    # Helper: read each sensor (optionally with simple debouncing)
    def read_value(sensor):
        if not stable:
            return sensor.value()
        # Majority vote across quick reads
        from time import sleep_ms
        ones = 0
        for _ in range(samples):
            ones += sensor.value()
            sleep_ms(delay_ms)
        return 1 if ones >= (samples // 2 + 1) else 0
    #end helper

    # count how many sensors detect fire
    expected = 0 if active_low else 1
    fire_count = sum(read_value(s) == expected for s in sensors)

    # total flames lit
    total_flames = fire_count + 1
    return total_flames
'''
CALL FUNCTION: numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4,stable=True) YOU CAN REMOVE STABLE TO REMOVE DEBOUNCING

BELOW PRINTS OUT RESULT OF FUNCTION
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")
'''

#ORIENTATION FUNCTION #HOW DO WE GET LEFT AND RIGHT DISTANCE
def classify_position(left_distance, right_distance): 
    # left sesnor is sesnor #4
    # right sesnor is sensor #5
    # ranges for each option
    option_A = (9.4 <= left_distance <= 9.7) and (6.7 <= right_distance <= 7.3)
    option_B = (11.8 <= left_distance <= 12.3) and (9.4 <= right_distance <= 9.6)
    option_C = (9.4 <= left_distance <= 9.7) and (9.4 <= right_distance <= 9.7)
    option_D = (6.7 <= left_distance <= 7.3) and (11.8 <= right_distance <= 12.3)

    # Check which option matches
    if option_A:
        return "Option A"
        # OTV  is facing side A
    elif option_B:
        return "Option B"
        # OTV  is facing side B 
    elif option_C:
        return "Option C"
        # OTV  is facing side C
    elif option_D:
        return "Option D"
        # OTV  is facing unlabeld side 
    else:
        return "Unknown — values do not match any option."
#VICTOR DANCE
def victory_dance():
    move(.5,100,100)
    move(.3,-50,100)
    move(.3,-100,-100)
    move(.3,-50,100)
    move(.3,50,-100)
    #i think shimining

#NAVIGATION RUN
def update_position():
    global x, y, theta
    while True:
        time.sleep(1)   # update every second
        x = enes100.x()
        y = enes100.y()
        theta = enes100.theta()

# start background pose updater
_thread.start_new_thread(update_position, ())
# state matchine definitions
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
                    # spin tiny bit
                    #call updateposition
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

#Victory Dance?
