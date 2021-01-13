import time
import serial
import math
from math import * # fix
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
from config import OPTICAL_DPI, ARDUINO_COM_PORT
import csv
from imu import IMU

# init IMU connection
imu = IMU()

# reset main.csv
with open('main.csv', 'w') as f:
    fieldnames = ['timestamp', 'pose_x', 'pose_y', 'pose_yaw', 'dx', 'dy', 'loop']
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

file_position = 0
# returns optical displacement vector since last call
def get_displacement_vector():
    global file_position
    try:
        with open("mouse.data", "r") as f:
            lines = f.readlines()[file_position:]
            file_position = file_position+len(lines)
            x = 0
            y = 0
            for line in lines:
                dx, dy = line.split(',')
                x = x + int(dx)
                y = y - int(dy)
            return (x/OPTICAL_DPI, y/OPTICAL_DPI)
    except IOError:
        # this should never happen
        # fix if it does
        print("mouse.data unavailable")
        return (0, 0)

# returns two angles whose difference is minimized: (a,b)
# assumes difference is +-180
def shortest_angle(angle_1, angle_2):
    diff = angle_2-angle_1
    return (angle_1, angle_2-360) if diff > 180 else (angle_1-360, angle_2) if diff < -180 else (angle_1, angle_2)


# pose exponential using numerical integration
# assuming constant curvature, angular rate, and velocity
# TODO: upload derivation
def global_translation(x, y, theta_1_deg, theta_2_deg):
    a,b = shortest_angle(theta_1_deg, theta_2_deg)
    a = radians(a)
    b = radians(b)
    delta_theta = b-a

    if delta_theta == 0:
        return (
            y*cos(a) + x*cos(a-pi/2),
            y*sin(a) + x*sin(a-pi/2)
        )
    return (
        ((sin(b)-sin(a))*y+(cos(a)-cos(b))*x)/delta_theta,
        -((cos(b)-cos(a))*y+(sin(b)-sin(a))*x)/delta_theta
    )

# constants
# ALPHA = 0.15
RADIUS = 4.5/2.54 # 4.5 cm to in
YAW_ZERO = imu.get_euler_angles()[2]
print("IMU zeroed at {} deg.".format(YAW_ZERO))

# x,y,heading
pose = (0,0,0)
ts = time.perf_counter()

def main():
    global RADIUS
    global pose
    global ts
    global imu

    # might need to do time matching
    # optical waveform is fine but varies a lot with time
    while True:
        imu.update()
        new_yaw = imu.get_euler_angles()[2]
        if new_yaw != pose[2]:
            optical = get_displacement_vector()# we might be reading this even tho yaw hasn't changed so we're losing information
            dP = global_translation(optical[0], optical[1], pose[2], new_yaw)
            # dP = global_translation(optical[0], 0, pose[2], new_yaw)
            # dP = global_translation(math.hypot(optical[0], optical[1]), 0, pose[2], new_yaw)
            pose = (pose[0]+dP[0], pose[1]+dP[1], new_yaw)

            with open('main.csv', 'a') as f:
                writer = csv.writer(f, delimiter=',')
                now = time.perf_counter()
                writer.writerow([now, pose[0], pose[1], pose[2], optical[0], optical[1], (now-ts)*1000])
                ts = now
            
            # print(pose)

if __name__ == "__main__":
    main()