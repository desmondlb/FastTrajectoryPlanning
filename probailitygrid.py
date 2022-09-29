import random
import numpy as np
nums = [0,1,"X"]
grid = [[random.choices(nums, weights=(1749,750,1), k=2500) for x in range(50)] for y in range(50)]
#grid[][] = "X" For target
print(grid)
#Assuming 1 as unblocked

