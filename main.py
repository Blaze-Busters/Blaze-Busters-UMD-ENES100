from enes100 import enes100
import time

enes100.begin("Blaze Busters","FIRE",222,1120)

while (enes100.is_visible):
    print(enes100.x)
    print(enes100.y)
    print(enes100.theta)
    print("-----------------------------")
    time.sleep(2)
