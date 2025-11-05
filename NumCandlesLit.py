from machine import Pin, PWM
import time

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



#-------- Michael tried to add pins if it sucks ignore lmao...i did not touch sofija's function above jic

from machine import Pin
import time

# Wiring
LEFT_FLAME_GPIO  = 17
RIGHT_FLAME_GPIO = 25
FRONT_FLAME_GPIO = 26
BACK_FLAME_GPIO  = 39

# Pin Objects
FS1 = Pin(LEFT_FLAME_GPIO,  Pin.IN)
FS2 = Pin(RIGHT_FLAME_GPIO, Pin.IN)
FS3 = Pin(FRONT_FLAME_GPIO, Pin.IN)
FS4 = Pin(BACK_FLAME_GPIO,  Pin.IN)

# main func
def number_of_flames_lit(left_flame, right_flame, front_flame, back_flame, *, active_low=True, stable=False, samples=5, delay_ms=2): #i asked gpt to add debouncing, resulted in more parameters

    sensors = [left_flame, right_flame, front_flame, back_flame]

    # Helper: read each sensor (optionally with simple debouncing)
    def read_value(sensor):
        if not stable:
            return sensor.value()
        # Majority vote across quick reads
        from time import sleep_ms
        ones = 0
        for _ in range(samples):
            ones += sensor.value()
            sleep_ms(delay_ms)
        return 1 if ones >= (samples // 2 + 1) else 0
    #end helper

    # count how many sensors detect fire
    expected = 0 if active_low else 1
    fire_count = sum(read_value(s) == expected for s in sensors)

    # total flames lit
    total_flames = fire_count + 1
    return total_flames

#can we PLS somehow make it respond in emojis...like if 3 candles lit it prints three fire emojis...ye ima do that (might win some design award or smth)

numberLit = number_of_flames_lit(FS1, FS2, FS3, FS4)
fire_emoji = "🔥" * numberLit
print(f"Candles Lit: {fire_emoji}")

#ITS PERFECT


