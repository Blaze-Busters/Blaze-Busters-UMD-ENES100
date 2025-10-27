'''
READ ME
The way refresh_pose works is that it is a function that updates the position of the OTV every time it is called. 
You have to manually call it in every while loop where position is important. 
The benefit of the function is that it calls all three positions at once. 
I have not put this function in most while loops bcs it would get ugly to read. 
Also please check if the code correctly switches between zones :)
'''


import time
from enes100 import enes100
from machine import Pin, PWM

UPDATE_MS = 1000 #update position every 1 second
#zone maxes
Z1_MAX = 0.80
Z2_MAX = 2.80
Z3_MAX = 3.80

button_start = Pin(16, Pin.IN, Pin.PULL_UP) #rewire and stuff

# pose cache
x = y = theta = None
_last_update = 0

def refresh_pose():
    global x, y, theta, _last_update
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_update) >= UPDATE_MS:
        x = enes100.x()
        y = enes100.y()
        theta = enes100.theta()
        _last_update = now

# defined states and when it switches
IDLE, ZONE1, ZONE2, ZONE3, DONE = range(5)
state = IDLE

while True:
    refresh_pose()
    time.sleep_ms(10)

    # start once
    if state == IDLE and button_start.value() == 0:
        state = ZONE1

    if state == ZONE1: #Zone 1
         while (x < 0.8):
           if (y < 1.3):
               while not (1.541 < theta < 1.599):  # face +pi/2
                   #spin to face correct direction
               # move forward until given coordinate
           else:
               while not (-1.599 < theta < -1.541):  # face -pi/2
                   #spin to face correct diretion
                   
               # move forward until given coordinate
           # extinguish candles etc.
        #------------------------------------------
        # transition condition:
        if x is not None and x >= Z1_MAX:
            state = ZONE2

    elif state == ZONE2: #Zone 2
         while (0.79 < x < 2.8):
           while (ultrasound_front > 10):
               # move forward
           if (ultrasound_front < 10):
               if (y > 1.5):
                   # right 90, forward until left clear, left 90
               elif (y > 0.8):
                   # same
               else:
                   # left 90, forward until left clear, right 90
           # repeat pattern above 2 more times
        # ------------------------------------------
        if x is not None and x >= Z2_MAX:
            state = ZONE3

    elif state == ZONE3: #Zone 3
         while (2.79 < x < 3.8):
           if (y < 1.5):
               # turn left 90
               while (y < 1.5):
                   # move forward
           else:
               # move forward
        # ------------------------------------------
        if x is not None and x >= Z3_MAX:
            state = DONE

    elif state == DONE:
        # finished/stop motors?
        pass
