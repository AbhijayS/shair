import os
import time
import serial
import math
from math import *  # fix
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from multiprocessing import Process
from config import OPTICAL_DPI, ARDUINO_COM_PORT
import csv
from imu import IMU
import struct
import numpy as np
import sys

# init IMU connection
imu = IMU()

# reset clipper.csv
log_file = 'clipper.csv'
log_fieldnames = ['timestamp', 'pose2d', 'pose3d',
                  'units', 'triangle', 'optical', 'loop']
with open(log_file, 'w') as f:
    writer = csv.DictWriter(f, fieldnames=log_fieldnames)
    writer.writeheader()

file_position = 0
# returns optical displacement vector since last call


def get_displacement_vector():
    global file_position
    try:
        with open("mouse.csv", "r") as f:
            lines = f.readlines()[file_position:]
            file_position = file_position+len(lines)
            x = 0
            y = 0
            for line in lines:
                dx, dy = line.split(',')
                x = x + int(dx)
                y = y - int(dy)
            return [x/OPTICAL_DPI, y/OPTICAL_DPI]
    except IOError:
        # this should never happen
        # fix if it does
        print("mouse.csv unavailable")
        return [0, 0]

# returns two angles whose difference is minimized: (a,b)
# assumes max difference is +-180


def shortest_angle(angle_1, angle_2):
    diff = angle_2-angle_1
    return (angle_1, angle_2-360) if diff > 180 else (angle_1-360, angle_2) if diff < -180 else (angle_1, angle_2)


# pose exponential using numerical integration
# assuming constant curvature, angular rate, and speed
# TODO: upload derivation
def translation_2d(x, y, theta_1_deg, theta_2_deg):
    a, b = shortest_angle(theta_1_deg, theta_2_deg)
    a = radians(a)
    b = radians(b)
    delta_theta = b-a
    if delta_theta == 0:
        return [
            y*cos(a) + x*cos(a-pi/2),
            y*sin(a) + x*sin(a-pi/2)
        ]
    return [
        ((sin(b)-sin(a))*y+(cos(a)-cos(b))*x)/delta_theta,
        -((cos(b)-cos(a))*y+(sin(b)-sin(a))*x)/delta_theta
    ]

# TODO: speed up reading of large files
#       checking for redundant points is too inefficient
#           maybe try constructing triangles using redundant points?


def read_stl(filename):
    '''
    returns:
        - tuple of points, normals, and triangles
        - points, normals, and triangles are numpy arrays
        - points.shape => (num_points, 3)
        - normals.shape => (num_normals, 3)
        - triangles.shape => (num_triangles, 3)
            - each triangle contains 3 point indices (referencing the points array)
    '''

    with open(filename, 'rb') as f:
        f.seek(80)
        [facets, ] = struct.unpack('I', f.read(4))
        points = []
        triangles = []
        normals = []

        for i in range(facets):
            normal = list(struct.unpack('fff', f.read(12)))
            normals.append(normal)

            v1 = list(struct.unpack('fff', f.read(12)))
            v2 = list(struct.unpack('fff', f.read(12)))
            v3 = list(struct.unpack('fff', f.read(12)))
            triangle = []

            if v1 in points:
                triangle.append(points.index(v1))
            else:
                triangle.append(len(points))
                points.append(v1)

            if v2 in points:
                triangle.append(points.index(v2))
            else:
                triangle.append(len(points))
                points.append(v2)

            if v3 in points:
                triangle.append(points.index(v3))
            else:
                triangle.append(len(points))
                points.append(v3)

            triangles.append(triangle)
            f.seek(2, 1)
    return (np.array(points), np.array(normals), np.array(triangles))


def transform_coordinates(local_units, global_vector):
    # params:
    # local_units := local unit vectors expressed using the global system
    # global_vector := vector to be converted into local system
    # returns: local vector coordinates in local units
    global_units = np.array([
        [1, 0, 0],  # i-hat
        [0, 1, 0],  # j-hat
        [0, 0, 1]  # k-hat
    ])
    return np.matmul(np.dot(global_units, local_units), global_vector)


def point_local_to_global(local_units, origin_point, local_point):
    return origin_point + local_units[0]*local_point[0] + local_units[1]*local_point[1] + local_units[2]*local_point[2]


def vector_local_to_global(local_units, local_vector):
    return np.sum(local_units*local_vector, axis=0)


def intersection_point(p1, p2, p3, p4):
    # params:
    # line 1: p1(x1,y1), p2(x2,y2)
    # line 2: p3(x3,y3), p4(x4,y4)
    # returns
    # (int, (x,y))
    # int:
    # 0 if no intersection
    # 1 if 1 intersection
    # 2 if coincident
    # x,y: x-y coordinates of intersection point
    denom = (p4[1]-p3[1])*(p2[0]-p1[0])-(p4[0]-p3[0])*(p2[1]-p1[1])
    numa = (p4[0]-p3[0])*(p1[1]-p3[1])-(p4[1]-p3[1])*(p1[0]-p3[0])
    numb = (p2[0]-p1[0])*(p1[1]-p3[1])-(p2[1]-p1[1])*(p1[0]-p3[0])
    if denom == 0:
        # parallel
        if numa == 0 and numb == 0:
          # coincident
            return (2,)
        return (0,)
    else:
        ua = numa/denom
        ub = numb/denom
        if ua > 0 and ua < 1 and ub > 0 and ub < 1:
            x = p1[0] + ua*(p2[0]-p1[0])
            y = p1[1] + ua*(p2[1]-p1[1])
            return (1, (x, y))
        return (0,)


def adjacent_triangle(tri, ends):
    # tri and ends contain point indices
    # returns adj triangle index and its normal
    global triangles

    for p in tri:
        if not p in ends:
            third = p

    for i in range(len(triangles)):
        t = triangles[i]
        if (ends[0] in t) and (ends[1] in t) and (not third in t):
            n = normals[i]
            return (i, n)

# https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula


def rodrigues_rotation(n1, n2, v):
    cross = np.cross(n1, n2)
    if not cross.any():
        return v
    a = cross / np.linalg.norm(cross)
    sinphi = np.linalg.norm(
        np.cross(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2)))
    cosphi = np.dot(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2))
    return v*cosphi + np.cross(a, v)*sinphi + a*np.dot(a, v)*(1-cosphi)


def translation_3d(movement_2d, delta_yaw):
    global location
    global unit_vectors
    global origin
    global vertices
    global v0
    global v1
    global v2
    global u
    global v
    global w
    global triangle

    move = 1
    rotate = -math.radians(5)

    previous_intersection_edge = None
    movement_vector = np.array(movement_2d+[0])

    while not np.array_equal(movement_vector, [0, 0, 0]):
        # calculate intersections
        # edge1: v0 -> v1
        if not np.array_equal(previous_intersection_edge, [v1, v0]):
            edge1_p0 = transform_coordinates(unit_vectors, v0-origin)
            edge1_p1 = transform_coordinates(unit_vectors, v1-v0) + edge1_p0
            i1 = intersection_point(
                location, location+movement_vector, edge1_p0, edge1_p1)
        else:
            i1 = (0,)

        # edge2: v1 -> v2
        if not np.array_equal(previous_intersection_edge, [v2, v1]):
            edge2_p0 = transform_coordinates(unit_vectors, v1-origin)
            edge2_p1 = transform_coordinates(unit_vectors, v2-v1) + edge2_p0
            i2 = intersection_point(
                location, location+movement_vector, edge2_p0, edge2_p1)
        else:
            i2 = (0,)

        # edge3: v2 -> v0
        if not np.array_equal(previous_intersection_edge, [v0, v2]):
            edge3_p0 = transform_coordinates(unit_vectors, v2-origin)
            edge3_p1 = transform_coordinates(unit_vectors, v0-v2) + edge3_p0
            i3 = intersection_point(
                location, location+movement_vector, edge3_p0, edge3_p1)
        else:
            i3 = (0,)

        # print("mov:" + str(movement_vector))
        # print("tri: " + str(triangle))
        # print("prev:" + str(previous_intersection_edge))
        # print(i1,i2,i3)
        # print("")

        if i1[0] == 1 and i2[0] != 1 and i3[0] != 1:
            previous_intersection_edge = [v0, v1]

            # update movement vector
            movement_vector = movement_vector-(i1[1]+(0,))

            # update origin point to intersection point
            origin = point_local_to_global(unit_vectors, origin, i1[1]+(0,))

            # calculate new triangle and vertices
            triangle, new_norm = adjacent_triangle(
                vertices, [vertices[0], vertices[1]])
            vertices = triangles[triangle]
            v0, v1, v2 = map(lambda v: points[v], vertices)

            # calculate new unit vectors on new triangle
            unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(
                unit_vectors[2], new_norm, x), unit_vectors)))
            u, v, w = unit_vectors

        # axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[0], color='r')
        # axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[1], color='g')

        elif i2[0] == 1 and i1[0] != 1 and i3[0] != 1:
            previous_intersection_edge = [v1, v2]

            # update movement vector
            movement_vector = movement_vector-(i2[1]+(0,))

            # update origin point to intersection point
            origin = point_local_to_global(unit_vectors, origin, i2[1]+(0,))

            # calculate new triangle and vertices
            triangle, new_norm = adjacent_triangle(
                vertices, [vertices[1], vertices[2]])
            vertices = triangles[triangle]
            v0, v1, v2 = map(lambda v: points[v], vertices)

            # calculate new unit vectors on new triangle
            unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(
                unit_vectors[2], new_norm, x), unit_vectors)))
            u, v, w = unit_vectors

        elif i3[0] == 1 and i1[0] != 1 and i2[0] != 1:
            previous_intersection_edge = [v2, v0]

            # update movement vector
            movement_vector = movement_vector-(i3[1]+(0,))

            # update origin point to intersection point
            origin = point_local_to_global(unit_vectors, origin, i3[1]+(0,))

            # calculate new triangle and vertices
            triangle, new_norm = adjacent_triangle(
                vertices, [vertices[2], vertices[0]])
            vertices = triangles[triangle]
            v0, v1, v2 = map(lambda v: points[v], vertices)

            # calculate new unit vectors on new triangle
            unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(
                unit_vectors[2], new_norm, x), unit_vectors)))
            u, v, w = unit_vectors

        elif i3[0] == 0 and i2[0] == 0 and i3[0] == 0:
            previous_intersection_edge = None
            origin = point_local_to_global(
                unit_vectors, origin, movement_vector)
            movement_vector = [0, 0, 0]

        else:
            print("uh oh")
            break


points, normals, triangles = read_stl('tests/sphere.stl')
# 3d stuff

# starting triangle and vertices
triangle = 0
vertices = triangles[triangle]
v0 = points[vertices[0]]
v1 = points[vertices[1]]
v2 = points[vertices[2]]

# initial coordinate system
# created at v0 of triangle
# unit vectors: uvw
origin = v0
w = np.array(normals[triangle])
v = (v1-v0) / np.linalg.norm(v1-v0)
u = np.cross(v, w) / np.linalg.norm(np.cross(v, w))
unit_vectors = np.array([u, v, w])

# initial starting location
# in reference to uvw units
location = np.array([-0.003, 0.01, 0.])

# move origin to new location
origin = point_local_to_global(unit_vectors, origin, location)

# set location back to zero (origin)
# always zero (relative to origin)
location = np.array([0., 0., 0.])

# 2d stuff
YAW_ZERO = imu.get_euler_angles()[2]
pose2d = (0, 0, 0)  # x,y,yaw
ts = time.perf_counter()  # timing performance
print("IMU zeroed at {} deg.".format(YAW_ZERO))


def main():
    global pose2d
    global ts
    global imu
    global loc
    global unit
    global tri
    global origin
    global unit_vectors
    global triangle
    global log_file
    global log_fieldnames

    # NOTE: might need to do time matching to get imu and optical data to match up
    #       though right now it doesn't seem necessary
    while True:
        imu.update()

        old_yaw = pose2d[2]
        new_yaw = imu.get_euler_angles()[2]
        motion = get_displacement_vector()
        # dP = translation_2d(motion[0], motion[1], old_yaw, new_yaw)
        dP = translation_2d(motion[0], motion[1], old_yaw, old_yaw)
        # pose2d = (pose2d[0]+dP[0], pose2d[1]+dP[1], new_yaw)
        pose2d = (pose2d[0]+dP[0], pose2d[1]+dP[1], old_yaw)
        translation_3d(dP, 0)

        # TODO: update guard position here

        # print("tri: " + str(tri))
        # print("loc: " + str(loc))
        # print("2d: " + str(pose2d))
        # os.system('cls')
        # print(dP)

        with open(log_file, 'a') as f:
            writer = csv.DictWriter(
                f, delimiter=',', fieldnames=log_fieldnames)
            now = time.perf_counter()
            # writer.writerow([now, *pose2d, *loc, 0, *motion, (now-ts)*1000])
            writer.writerow({
                'timestamp': now,
                'pose2d': pose2d,
                'pose3d': repr(origin).replace('\n', ''),
                'units': repr(unit_vectors).replace('\n', ''),
                'triangle': triangle,
                'optical': motion,
                'loop': (now-ts)*1000
            })
            ts = now


if __name__ == "__main__":
    main()
