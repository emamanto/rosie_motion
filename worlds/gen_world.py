import random
import sys
import time
import math

num_blocks = int(sys.argv[1])

print("Making world with " + str(num_blocks) + " blocks to sort")

fileshort = "env" + time.strftime("%H%M%S", time.gmtime())
filename = fileshort + ".sdf"
world_file = open(filename, 'w')

x_min = 0.6
x_max = 0.9
y_min = -0.3
y_max = 0.3

yaw_min = -math.pi
yaw_max = math.pi

empty_table = list(open('empty_table.sdf', 'r'))
for line in empty_table:
    if line.find("</world>") != -1:
        break
    world_file.write(line)

block_template = list(open('obj_template.txt'))

block_poses = []

color = "blue"
for n in range(0, num_blocks):
    if color == "blue":
        color = "red"
    else:
        color = "blue"

    size_check = random.random()
    size = "none"
    size_m = 0
    if size_check <= 1.0/6.0:
        size = "3"
        size_m = 0.03
    elif size_check > 1.0/6.0 and size_check <= 2.0/6.0:
        size = "5"
        size_m = 0.05
    elif size_check > 2.0/6.0 and size_check <= 3.0/6.0:
        size = "7"
        size_m = 0.07
    elif size_check > 3.0/6.0 and size_check <= 4.0/6.0:
        size = "9"
        size_m = 0.09
    elif size_check > 4.0/6.0 and size_check <= 5.0/6.0:
        size = "11"
        size_m = 0.11
    elif size_check > 5.0/6.0:
        size = "13"
        size_m = 0.13

    valid = False
    block_x = 0
    block_y = 0
    while not valid:
        block_x = (x_max-x_min)*random.random() + x_min
        block_y = (y_max-y_min)*random.random() + y_min

        valid = True
        for pos in block_poses:
            v = [block_x - pos[0], block_y - pos[1]]
            dist = math.sqrt(math.pow(v[0], 2) + math.pow(v[1], 2))
            if dist < size_m + pos[2]:
                valid = False
                break

    block_poses.append([block_x, block_y, size_m])
    block_yaw = (yaw_max-yaw_min)*random.random() + yaw_min

    for line in block_template:
        if line.find("COLOR") != -1:
            first = line[:line.find("COLOR")]
            third = line[line.find("COLOR")+5:line.find("SIZE")]
            last = line[line.find("SIZE")+4:]
            world_file.write(first + color + third + size + last)
        elif line.find("NUM") != -1:
            first = line[:line.find("NUM")]
            third = line[line.find("NUM")+3:]
            world_file.write(first + str(n+1) + third)
        elif line.find("POSE") != -1:
            first = line[:line.find("POSEX")]
            third = line[line.find("POSEX")+5:line.find("POSEY")]
            fifth = line[line.find("POSEY")+5:line.find("YAW")]
            last = line[line.find("YAW")+3:]
            world_file.write(first + str(block_x) + third + str(block_y) +
                             fifth + str(block_yaw) + last)
        else:
            world_file.write(line)

do_write = False
for line in empty_table:
    if line.find("</world>") != -1:
        do_write = True
    if do_write:
        world_file.write(line)
