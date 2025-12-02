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
    option_A = (9.4 <= left_distance <= 9.7) and (6.7 <= right_distance <= 7.3)
    option_B = (11.8 <= left_distance <= 12.3) and (9.4 <= right_distance <= 9.6)
    option_C = (9.4 <= left_distance <= 9.7) and (9.4 <= right_distance <= 9.7)
    option_D = (6.7 <= left_distance <= 7.3) and (11.8 <= right_distance <= 12.3)

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

# Start background thread
_thread.start_new_thread(update_sensors, ())

# Allow thread to initialize
time.sleep(0.2)

# Main loop
while True:
    print("Left distance:", left_sensor_down)
    print("Right distance:", right_sensor_down)
    #print("Position:", classify_position(left_sensor_down, right_sensor_down))
    print("-----")
    time.sleep(0.3)
