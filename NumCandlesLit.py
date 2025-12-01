from machine import Pin, PWM
import time

#FLAME SENSOR PINS
FS1 = Pin(17, Pin.IN)
FS2 = Pin(25, Pin.IN)
FS3 = Pin(26, Pin.IN)
FS4 = Pin(39, Pin.IN)

def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame):

    # list of all sensors
    sensors = [left_flame, right_flame, front_flame, back_flame]
    # count how many sensors detect fire (value == 0)
    fire_count = sum(1 for sensor in sensors if sensor.value() == 0)
    # total flames lit
    total_flames = 1 + fire_count
    return total_flames

# Call function example
numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4)
print("Total Number of Candles Lit: " + numberLit)



