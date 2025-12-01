import time
from time import sleep_us, sleep
import _thread
from enes100 import enes100
from machine import Pin, time_pulse_us, ADC, PWM

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

#sensors
front_sensor = 0
left_sensor_side = 0
right_sensor_side = 0
left_sensor_down = 0
right_sensor_down = 0

#Sensor Pins
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

def distance_cm(trig, echo):
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)
    dist_cm = (duration / 2) * 0.0343
    return dist_cm

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

#NUM CANDLES LIT FUNCTION
def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame):
    # list of all sensors
    sensors = [left_flame, right_flame, front_flame, back_flame]
    # count how many sensors detect fire (value == 0)
    fire_count = sum(1 for sensor in sensors if sensor.value() == 0)
    # total flames lit
    total_flames = 1 + fire_count
    return total_flames

#BLOCK ORIENTATION FUNCTION
def classify_position(left_distance, right_distance):
    # left sesnor is sesnor #4
    # right sesnor is sensor #5
    # ranges for each option
    option_A = (9.4 <= left_distance <= 9.7) and (6.7 <= right_distance <= 7.3)
    option_B = (11.8 <= left_distance <= 12.3) and (9.4 <= right_distance <= 9.6)
    option_C = (9.4 <= left_distance <= 9.7) and (9.4 <= right_distance <= 9.7)
    option_D = (6.7 <= left_distance <= 7.3) and (11.8 <= right_distance <= 12.3)

#SERVO PIN
servo = PWM(Pin(16), freq=50)
#SERVO FUNCTION (DROP AND LIFT SNUFFS)
def Snuffs(duration, speed):
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


    # Check which option matches
    if option_A:
        return "Side A"
        # OTV  is facing side A
    elif option_B:
        return "Side B"
        # OTV  is facing side B 
    elif option_C:
        return "Side C"
        # OTV  is facing side C
    elif option_D:
        return "Side D"
        # OTV  is facing unlabeld side 
    else:
        return "Unknown — values do not match any option." #gulp

motor_left = DCMotor(IN1, IN2, ENA)
motor_right = DCMotor(IN3, IN4, ENB)


class DCMotor:
    def __init__(self, in_a, in_b, en_pwm):
        self.in_a = in_a
        self.in_b = in_b
        self.en = en_pwm
        self.stop()

    def _set_speed(self, frac):
        if frac < 0: frac = 0
        if frac > 1: frac = 1
        try:
            self.en.duty_u16(int(frac * 65535))
        except:
            self.en.duty(int(frac * 1023))

    def forward(self, speed):
        self.in_a.value(1)
        self.in_b.value(0)
        self._set_speed(speed)

    def stop(self):
        self.in_a.value(0)
        self.in_b.value(0)
        self._set_speed(0)

def move_spin(duration, speed_left, speed_right):
    """
    speed_left: 0.0 to 1.0
    speed_right: 0.0 to 1.0
    """
    # Set motors to forward direction
    motor_left.forward(speed_left)
    motor_right.forward(speed_right)

    time.sleep(duration)

    # Stop motors after movement
    motor_left.stop()
    motor_right.stop()



x = 0
y = 0
theta = 0

def update_pose():
    global x, y, theta
    x = enes100.x()
    y = enes100.y()
    theta = enes100.theta()

#START
update_position()

if y < 1.3:
    # face +pi/2 (1.541 to 1.599 rad)
    while not (1.541 < theta < 1.599):
        motors_spin(1, 50, -50)
        update_pose()
        pass
    
                
else:
    # face -pi/2 (-1.599 to -1.541 rad)
    while not (-1.599 < theta < -1.541):
        motors_spin(1, 50, -50)
        update_pose()
        pass

 #loop, go till reach sensor hits mission block
while(front_sensor<5)
    motors_spin(3, 50, 50)
    update_pose()
    pass

update_pose()

#print orientation
print(classify_position(left_sensor_down, right_sensor_down))
#Print total number of flames lit, (includes middle flame)
print("Total number of Flames Lit ", number_of_flames_lit(FS1, FS2, FS3, FS4))






