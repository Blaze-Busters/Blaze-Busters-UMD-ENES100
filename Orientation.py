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
        return "Unknown — values do not match any option." #gulp


left_sensor_down = 0
right_sensor_down = 0

TRIG3 = Pin(14, Pin.OUT)
ECHO3 = Pin(34, Pin.IN)
TRIG4 = Pin(27, Pin.OUT)
ECHO4 = Pin(35, Pin.IN)

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
    global left_sensor_down, right_sensor_down
    while True:
        left_sensor_down = distance_cm(TRIG3, ECHO3)
        right_sensor_down = distance_cm(TRIG4, ECHO4)

        time.sleep(0.1)   # Update at 10Hz
        
_thread.start_new_thread(update_sensors, ())

print("left distance: " + left_sensor_down)
print("right distance: " + right_sensor_down)

print(classify_position(left_sensor_down, right_sensor_down))

