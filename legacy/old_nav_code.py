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
                    motor_off()n

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
