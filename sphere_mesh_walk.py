from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot
import numpy as np
import trimesh
import math

# Create a new plot
figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

# Load the STL files and add the vectors to the plot
# your_mesh = mesh.Mesh.from_file('sphere.stl')
# axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

# Auto scale to the mesh size
# print(your_mesh.points)
# scale = your_mesh.points.flatten(-1)
# axes.auto_scale_xyz(scale, scale, scale)


import struct

with open('sphere.stl', 'rb') as f:
    f.seek(80)
    [facets,] = struct.unpack('I', f.read(4))
    points = []
    triangles = []
    normals = []

    for i in range(facets):
        normal = list(struct.unpack('fff', f.read(12)))
        v1 = list(struct.unpack('fff', f.read(12)))
        v2 = list(struct.unpack('fff', f.read(12)))
        v3 = list(struct.unpack('fff', f.read(12)))
        normals.append(normal)
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

points = np.array(points)
triangles = np.array(triangles)

# print(points)
# print(triangles)
    # print(facets)
    # print(normal)
    # print(v1)
    # print(v2)
    # print(v3)

# print(al_x)

# axes.scatter(points[0][0], points[0][1], points[0][2], c='r', s=100)
z = normals[0]
x = (points[0] - points[1])/math.hypot(*(points[0] - points[1])) # normalize
y = (np.cross(x, z))/math.hypot(*np.cross(x, z))
print(x,y,z)

axes.quiver(*points[0], *x, color='r')
axes.quiver(*points[0], *y, color='g')
axes.quiver(*points[0], *z, color='b')

axes.plot_trisurf(points[:,0], points[:,1], triangles, points[:,2])
pyplot.show()
# print(points[:,2])

# print(points[0])
# print(points[1])
# print(points[0]-points[1])
# how to travel
# local_coordinate = rotate global xyz axes according to unit normal vector
# if point of intersection:
    # 
# update lcoation in xyz on the same plane