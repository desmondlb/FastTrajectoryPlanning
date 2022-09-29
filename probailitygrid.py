import random
import numpy as np
nums = [0,1]
grid = [[random.choices(nums, weights=(70,30), k=2500) for x in range(50)] for y in range(50)]
print(grid)
#Assuming 1 as unblocked

