#-----Run this code Wednesday
while(front_sensor>20): #gets us into box
    update_sensors()
    motor_on(-50,-100)
    time.sleep(.05)
motor_off()
motors_spin(1,-20,-20) #snugs into bux

#detects how many candles are lit
numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4, stable=True)
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")

#puts out candles
spin(2, 50)
time.sleep(2)
spin(2,-50)

#checks orientation
print(classify_position(left_sensor_down, right_sensor_down))

#back outs and turns
motors_spin(5,50,100)
time.sleep(0.2)
motors_spin(2.3,-50,100)


#-------------------------
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



