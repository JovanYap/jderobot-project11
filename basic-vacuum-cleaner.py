from GUI import GUI
from HAL import HAL
import random, math, time

state = 1
v = 0  
w = 5 

while True:
    if state == 1:  
        # Circular exploration
        if HAL.getBumperData().state == 0:
            HAL.setV(v)
            HAL.setW(w)  
            v+=0.02
            print("state 1")
        else:
            # If an obstacle is encountered, move to obstacle avoidance state
            state = 2 
            # print("state 2")
            v = 0 
            w = 3
            HAL.setV(v)
            turn_direction = 1 if HAL.getBumperData().bumper in [0, 1] else -1
            HAL.setW(w * turn_direction)
            time.sleep(random.random() * math.pi)
            HAL.setV(0)
            HAL.setW(0)  
            time.sleep(0.8) 
            state = 3

    elif state == 3: 
        # Resume straight exploration
        # print("state 3")
        if HAL.getBumperData().state == 0:
          v = 3
          w = 0
          HAL.setV(v)
          HAL.setW(w)
        else: 
          state = 1

    time.sleep(0.1)  # Small delay to ease the control loop