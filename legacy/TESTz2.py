elif state == ZONE2:
    while Z1_MAX < x < Z2_MAX:

        front = front_sensor
        left  = left_sensor_side
        right = right_sensor_side

        # ============================
        #   MAIN FORWARD CONDITION
        # ============================
        if front > 15 and left > 10 and right > 10:
            motors_spin(0.5, 50, 50)  # move forward
            continue

        # ============================
        #   SIDE OBSTACLE AVOIDANCE
        # ============================

        # ---- LEFT SIDE OBSTACLE ----
        if left < 10:
            # Move forward until left side clears
            while left_sensor_side < 10:
                motors_spin(0.3, 50, 50)

            # Extra forward
            motors_spin(0.5, 50, 50)

            # Turn RIGHT 90 degrees
            motors_spin(0.7, 50, -50)  

            # Move forward until front is clear
            while front_sensor < 15:
                motors_spin(0.3, 50, 50)

            # Turn LEFT 90 degrees to restore direction
            motors_spin(0.7, -50, 50)

            continue

        # ---- RIGHT SIDE OBSTACLE ----
        if right < 10:
            # Move forward until right clears
            while right_sensor_side < 10:
                motors_spin(0.3, 50, 50)

            motors_spin(0.5, 50, 50)

            # Turn LEFT 90 degrees
            motors_spin(0.7, -50, 50)

            # Move forward until front clears
            while front_sensor < 15:
                motors_spin(0.3, 50, 50)

            # Turn RIGHT 90 degrees
            motors_spin(0.7, 50, -50)

            continue

        # ============================
        #   FRONT OBSTACLE HANDLING
        # ============================
        if front < 10:
            # Upper half logic
            if y >= 1.5:
                # Try turning RIGHT first
                motors_spin(0.7, 50, -50)  # turn right 90
                continue

            # Lower half logic
            else:
                # Try turning LEFT first
                motors_spin(0.7, -50, 50)  # turn left 90
                continue
