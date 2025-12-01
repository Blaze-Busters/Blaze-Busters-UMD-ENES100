from machine import Pin, PWM
import time

#FLAME SENSOR PINS
FS1 = Pin(17, Pin.IN)
FS2 = Pin(25, Pin.IN)
FS3 = Pin(26, Pin.IN)
FS4 = Pin(39, Pin.IN)

servo = PWM(Pin(16), freq=50)
#SERVO FUNCTION (DROP AND LIFT SNUFFS)
def Snuffs(duration, speed):
    STOP_DUTY = 77
    MAX_RANGE = 25
    #speed
    speed = max(-100, min(100, speed))
    #convert speed (-100..100) → duty (52..102)
    duty = int(STOP_DUTY + (speed / 100) * MAX_RANGE)
    servo.duty(duty)
    #spin for desired duration
    time.sleep(duration)
    # Stop servo
    servo.duty(STOP_DUTY)


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

#Snuffs(5, 50)
#time.sleep(5)
#Snuffs(5,-50)


