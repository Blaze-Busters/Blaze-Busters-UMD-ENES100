from enes100 import enes100
from machine import Pin, PWM
button_start = Pin(16, Pin.IN, Pin.PULL_UP)
ultrasound_front = #value from front ultrasound sensor
ultrasound_left = #value from left ultrasound sensor
ultrasound_right = #value from right ultrasound sensor

while True: #figure out where to put it
    time.sleep(1)
    x = enes100.x()
    y = enes100.y()
    theta = enes100.theta()


    if(button_start):
        while(x < 0.8): #while in zone 1
            if(enes100.y<1.3): #if at bottom position
                while(!((theta>1.541)&&(theta<1.599))): #if not in wanted range
                    #spin in place to face candle (facing certain theta)
                #move forward until given cordinate
            else: #if at top start
                while(!((theta<-1.541)&&(theta>-1.599))):
                    #spin in place to face candle (facing certain theta))
                #move forward until given cordinate

            #do stuff to put out candles and all that fun stuff
            #back up 10 inches, turn 90 degress to face finish line
            #move forward 15 inches, turn left. Move forward until near wall, turn right
            #move forward
            
        while(<.79x<2.8): #while in zone 2
            while(ultrasound_front>10):
                #move forward

            #obstacle avoidance Round 1
            if(ultrasonic_front<10):
                if(y>1.5):
                    #turn right 90 degrees
                    while(ultrasound_left<=10):
                        #move forward
                    #turn left 90 degrees
                elif(y>0.8):
                    #turn right 90 degrees
                    while(ultrasound_left<=10):
                        #move forward
                    #turn left 90 degrees
                else:
                     #turn left 90 degrees
                     while(ultrasound_left<=10):
                         #move forward
                    #turn right 90 degrees

            #copy and paste this code 2 more times when finished :p
                    
        while(2.79<x<3.8): #zone3
            if(y<1.5): #if so its easier to write an else statement telling OTV to move forward if it is alr going to go under the bar
                #turn OTV left 90 degrees
                while(y<1.5):
                    #move forward
            else:
                #move forward
#DONE
