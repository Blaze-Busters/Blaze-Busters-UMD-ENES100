from enes100 import enes100
from machine import Pin, PWM
button_start = Pin(16, Pin.IN, Pin.PULL_UP)

while True:
    time.sleep(1)
    x = enes100.x()
    y = enes100.y()
    theta = enes100.theta()

    if(button_start):
        if(enes100.y<1.3): #if at bottom position
            while(!((theta>1.541)&&(theta<1.599))): #if not in wanted range
                #spin in place to face candle
            #move forward
        else:
            while(!((theta<-1.541)&&(theta>-1.599))):
                #spin in place to face candle
            #move foward


