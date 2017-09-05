import random
import sys
import time

num_blocks = int(sys.argv[1])

fileshort = "env" + time.strftime("%H%M%S", time.gmtime())
filename = fileshort + ".world"
world_file = open(filename, 'w')

for n in range(0, num_blocks)
