while front_sensor > 25:
    print(front_sensor)
    update_sensors()
    motor_on(-100, -100)
    time.sleep(0.05)
motor_off()

# turn left
motors_spin(2.7, 60, -60)
motor_off()
time.sleep(0.1)

update_sensors()
while right_sensor_side < 20:   # obstacle present on right side
    update_sensors()
    motor_on(-60, 60)
    time.sleep(0.05)
motor_off()

# turn right
motors_spin(2.7, -60, 60)
motor_off()
time.sleep(0.1)
