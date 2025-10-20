from enes100 import enes100
import time

enes100.begin(team_name: "Blaze Busters", team_type: "FIRE", aruco_id: 222, room_num: 1120)

while (enes100.is_visible):
    print(enes100.x)
    print(enes100.y)
    print(enes100.theta)
    print("-----------------------------")
    time.sleep(2)
