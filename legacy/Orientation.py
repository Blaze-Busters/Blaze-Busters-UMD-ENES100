import _thread
import time
from machine import Pin, time_pulse_us

# ===== Sensors =====
left_sensor_down = 0
right_sensor_down = 0

TRIG5 = Pin(5, Pin.OUT)
ECHO5 = Pin(36, Pin.IN)
TRIG4 = Pin(27, Pin.OUT)
ECHO4 = Pin(35, Pin.IN)

def distance_cm(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    duration = time_pulse_us(echo, 1, 30000)
    dist_cm = (duration / 2) * 0.0343
    return dist_cm

def classify_position(left_distance, right_distance):
    option_A = (3.5 <= left_distance <= 5.5) and (8 <= right_distance <= 12)
    option_B = (2.7 <= left_distance <= 5.2) and (4 <= right_distance <= 6.3)
    option_C = (4.3 <= left_distance <= 7) and (3.7 <= right_distance <= 4.9)
    option_D = (7 <= left_distance <= 30) and (2 <= right_distance <= 4)

    if option_A:
        return "Option A"
    elif option_B:
        return "Option B"
    elif option_C:
        return "Option C"
    elif option_D:
        return "Option D"
    else:
        return "Unknown — values do not match any option."

def update_sensors():
    global left_sensor_down, right_sensor_down
    while True:
        left_sensor_down = distance_cm(TRIG4, ECHO4)
        right_sensor_down = distance_cm(TRIG5, ECHO5)
        time.sleep(0.1)

# start thread
_thread.start_new_thread(update_sensors, ())

# give thread time to update values
time.sleep(0.2)


while True:
    print("left distance:", left_sensor_down)
    print("right distance:", right_sensor_down)
    print(classify_position(left_sensor_down, right_sensor_down))
    time.sleep(0.5)


