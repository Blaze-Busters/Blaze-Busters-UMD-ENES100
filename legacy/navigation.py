# ----------------- NAVIGATION RUN -----------------

#---------ZONE 1-----------
if y < 1.3: #if placed right side of field
  while not (1.74 < enes100.theta < 1.77):
    motor_on(-30,50) #spins until facing box
    time.sleep(.1)
    motor_off()
else: #if placed left side of field
  while not (-1.77 < enes100.theta < -1.74):
    motor_on(-30,50) #spins until facing box
    time.sleep(.1)
    motor_off()
time.sleep(1)

#gets us into box
update_sensors()
while(front_sensor>5): #moves into box
    update_sensors()
    motor_on(-68,-100)
    time.sleep(.05)
motor_off()
time.sleep(0.2)
motors_spin(2,-50,-50) #snugs into box
time.sleep(1)

#detects how many candles are lit
update_sensors()
numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4, stable=True)
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")
enes100.print(f"Candles Lit: {fire_emoji}")
enes100.mission('NUM_CANDLES', numberLit)

time.sleep(0.5)

#checks orientation of box
update_sensors()
print(classify_position(left_sensor_down, right_sensor_down))
enes100.print(classify_position(left_sensor_down, right_sensor_down))

time.sleep(0.2)

#puts out candles
spin(2, -50)
time.sleep(2)
spin(1.8,50)
time.sleep(0.1)

time.sleep(0.4)

#back outs and turns
motors_spin(5,50,100)
time.sleep(0.4)
motors_spin(2.4,-50,100) #check turn values during testing

#-----------ZONE 2-----------
update_sensors()
while (enes100.x<3): #while still in obstacle range
    update_sensors()

    #Gets us in front of an obstacle, stops before it
    while front_sensor > 37:
        update_sensors()
        motor_on(-63, -100)
        time.sleep(0.1)
    time.sleep(0.01)
    motors_off()
    time.sleep(0.5)
    update_sensors()
    
    #figuring out which way to turn
    if y<0.8:#if on right side of field
        motors_spin(2.6,50,-100) #turn left
        update_sensors()
        while right_sensor_side < 40: #move until past obstacle
            update_sensors()
            motor_on(-68, -100)
            time.sleep(0.05)
        motor_off()
        time.sleep(0.2)
        motors_spin(1.5,-63,-100) #turn right
        
    else:
        motors_spin(1.5,-63,100) #turn right
        update_sensors()
        while left_sensor_side < 40:
            update_sensors()
            motor_on(-68,-100)
            time.sleep(0.05)
        motor_off()
        time.sleep(0.2)
        motors_spin(2.6,50,-100)

#--------ZONE 3----------
if enes100.y < 1.7: #get in line to go under the bar
    motors_spin(2.6,50,-100) #turn left
    time.sleep(0.2)
    
    while enes100.y <1.5:
        motors_on(-63,-100)
        
    time.sleep(0.2)
    motors_off()
    time.sleep(0.2)
    motors_spin(1.5,-63,-100) #turn right
    time.sleep(0.2)
    
    #now OTV in line to go straight through bar
    while enes.100.x <3.7:
        motors_on(-63,100) #move below bar
    time.sleep(2)
else:
    while enes.100.x <3.7:
        motors_on(-63,100) #move below bar
    time.sleep(2)

#---------ZONE 4----------
victory_dance()
