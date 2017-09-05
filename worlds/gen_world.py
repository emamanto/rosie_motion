import random
import sys
import time

num_blocks = int(sys.argv[1])

print("Making world with " + str(num_blocks) + " blocks to sort")

fileshort = "env" + time.strftime("%H%M%S", time.gmtime())
filename = fileshort + ".world"
world_file = open(filename, 'w')

x_min = 0.6
x_max = 0.9
y_min = -0.3
y_max = 0.3

empty_table = list(open('empty_table.sdf', 'r'))
for line in empty_table:
    if line.find("</world>") != -1:
        break
    world_file.write(line)

block_template = list(open('obj_template.txt'))

for n in range(0, num_blocks):
    color = "BLUE"
    if (random.random() > 0.5):
        color = "RED"

    size_check = random.random()
    size = "none"
    if size_check <= 1.0/6.0:
        size = "3"
    elif size_check > 1.0/6.0 and size_check <= 2.0/6.0:
        size = "5"
    elif size_check > 2.0/6.0 and size_check <= 3.0/6.0:
        size = "7"
    elif size_check > 3.0/6.0 and size_check <= 4.0/6.0:
        size = "9"
    elif size_check > 4.0/6.0 and size_check <= 5.0/6.0:
        size = "11"
    elif size_check > 5.0/6.0:
        size = "13"

    for line in block_template:
        if line.find("COLOR") != -1:
            first = line[:line.find("COLOR")]
            third = line[line.find("COLOR")+5:line.find("SIZE")]
            last = line[line.find("SIZE")+4:]
            world_file.write(first + color + third + size + last)
        else:
            world_file.write(line)

do_write = False
for line in empty_table:
    if line.find("</world>") != -1:
        do_write = True
    if do_write:
        world_file.write(line)
