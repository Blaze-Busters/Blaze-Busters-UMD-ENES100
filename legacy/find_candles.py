if y < 1.3:
  while not (1.541 < theta < 1.599):
    update_position()
    motor_on(-50,100)
  time.sleep(.1)
  motor_off()
else:
  while not (-1.599 < theta < -1.541):
    update_position()
    motor_on(-50,100)
  time.sleep(.1)
  motor_off()
