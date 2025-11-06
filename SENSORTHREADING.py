from machine import Pin
import time
import _thread

def distance_cm(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    
    duration = machine.time_pulse_us(echo, 1, 30000)  # wait for echo HIGH max 30ms
    dist_cm = (duration / 2) * 0.0343
    return dist_cm


# Global sensor readings
front_sensor = 0
left_sensor_side = 0
right_sensor_side = 0
left_sensor_down = 0
right_sensor_down = 0


# Background sensor update loop
def update_sensors():
    global front_sensor, left_sensor_side, right_sensor_side, left_sensor_down, right_sensor_down
    while True:
        # Assign your real pin variables here
        front_sensor = distance_cm(TRIG5, ECHO5)
        left_sensor_side = distance_cm(TRIG1, ECHO1)
        right_sensor_side = distance_cm(TRIG2, ECHO2)
        left_sensor_down = distance_cm(TRIG3, ECHO3)
        right_sensor_down = distance_cm(TRIG4, ECHO4)

        time.sleep(0.1)   # Update at 10Hz


# Start the thread on ESP32
_thread.start_new_thread(update_sensors, ())
