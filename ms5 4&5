import time
from enes100 import enes100

enes100.begin("Blaze Busters","FIRE",67,1120)
while True:
  x = enes100.x
  theta = enes100.theta

  # Topography missions
  if x <= 1:
      enes100.mission('TOPOGRAPHY', 'TOP_A')
  elif 1 < x <= 2:
      enes100.mission('TOPOGRAPHY', 'TOP_B')
  elif x > 2:
      enes100.mission('TOPOGRAPHY', 'TOP_C')

  time.sleep(5)

  # Candle missions
  if 0 < theta < 1.57:
      enes100.mission('NUM_CANDLES', 1)
  elif 1.57 < theta < 3.14:
      enes100.mission('NUM_CANDLES', 2)
  elif -1.57 < theta < 0:
      enes100.mission('NUM_CANDLES', 4)
  elif -3.14 < theta < -1.57:
      enes100.mission('NUM_CANDLES', 3)

  time.sleep(5)


