# NOTE:
# make sure to have collected all data before running this script
# run this script from project root

import math


EXPECTED_DPI = 1130
distance = int(input('Straight line distance travelled in +y direction: '))

with open('mouse.csv', 'r') as f:
    lines = f.readlines()
    x = 0
    y = 0
    for l in lines:
        x = x + int(l.split(',')[0])
        y = y + int(l.split(',')[1])
    t = math.copysign(math.hypot(x,y), y)
    actual_dpi = abs(round(t/distance))

    print("X:  Count={}".format(x))
    print("Y:  Count={}".format(y))
    print("T:  Count={}".format(t))
    print("DPI: Expected={} | Calculated={} | Error={}%".format(EXPECTED_DPI, actual_dpi, ((actual_dpi-EXPECTED_DPI)/EXPECTED_DPI)*100))
    print("DEV: X={} | Y={}".format(x/t, (y-t)))