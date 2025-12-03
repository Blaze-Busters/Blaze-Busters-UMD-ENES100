while front_sensor > 37:
    print(front_sensor)
    update_sensors()
    motor_on(-63, -100)
    time.sleep(0.05)
motor_off()

time.sleep(3)

# turn left
motors_spin(2.6,50,-100)
time.sleep(0.1)

update_sensors()
while right_sensor_side < 40:   # obstacle present on right side
    update_sensors()
    motor_on(-68, -100)
    time.sleep(0.05)
motor_off()
time.sleep(0.2)
motors_spin(1.5,-63,-100)

time.sleep(.1)
# turn right
motors_spin(2.4,-50,100)
time.sleep(0.1)

#-------------------------------

while True:
    while front_sensor > 37:
        print(front_sensor)
        update_sensors()
        motor_on(-63, -100)
        time.sleep(0.05)
    motor_off()

    time.sleep(0.1)

    # turn left
    motors_spin(2.6,50,-100)
    time.sleep(0.1)

    update_sensors()

    while right_sensor_side < 40:   # obstacle present on right side
            update_sensors()
            motor_on(-68, -100)
            time.sleep(0.05)
    motor_off()
    time.sleep(0.2)
    motors_spin(1.5,-63,-100)

    time.sleep(.1)
        # turn right
    motors_spin(2.4,-50,100)
    time.sleep(0.1)

