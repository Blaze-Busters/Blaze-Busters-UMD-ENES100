import time
from time import sleep_us, sleep
import _thread
from enes100 import enes100
from machine import Pin, time_pulse_us, ADC, PWM


front_sensor = 0
left_sensor_side = 0
right_sensor_side = 0
left_sensor_down = 0
right_sensor_down = 0

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




