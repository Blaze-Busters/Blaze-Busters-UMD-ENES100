import enes100 from enes100
from machine import Pin, PWM
button_start = Pin(16, Pin.IN, Pin.PULL_UP)

if(button_start):
    if(enes100.x<1):
        if(enes100.y<2.5): #change until right before candle
            while(enes100.theta<.2||enes100.theta>.2):
                #spin in place to face candles
            while(-.2<enes100.theta<.2):
                #move forward
        if(enes100.y<2.5||enes100.y==3):
            #slowly approch candle
