from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot, tri
import numpy as np
import trimesh
import math


def transform_coordinates(local_units, global_vector):
  # params:
    # local_units := local unit vectors expressed using the global system
    # global_vector := vector to be converted into local system
  # returns: local vector coordinates in local units
  global_units = np.array([
    [1,0,0], # i-hat
    [0,1,0], # j-hat
    [0,0,1]  # k-hat
  ])
  return np.matmul(np.dot(global_units, local_units), global_vector)

def intersection_point(p1,p2,p3,p4):
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
    if abs(ua)<=1 and abs(ub)<=1:
      x = p1[0] + ua*(p2[0]-p1[0])
      y = p1[1] + ua*(p2[1]-p1[1])
      return (1, (x,y))
    return (0,)


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

current_vertices = triangles[0]
current_location = points[0]

# new xyz unit vectors
z = normals[0]
x = (points[0] - points[1])/math.hypot(*(points[0] - points[1]))
y = (np.cross(x, z))/math.hypot(*np.cross(x, z))

axes.quiver(*points[0], *x, color='r')
axes.quiver(*points[0], *y, color='g')
axes.quiver(*points[0], *z, color='b')

movement_vector = np.array([0*x, 1*y])
edge1_vector = points[current_vertices[0]]-points[current_vertices[1]]
print(edge1_vector)
edge1 = np.array([current_location,transform_coordinates(np.array([x,y,z]), edge1_vector) + current_location])
print(edge1)
print(intersection_point(movement_vector[0], movement_vector[1], edge1[0], edge1[1]))

mask = [False]*len(triangles)
mask[0] = True
t = tri.Triangulation(points[:,0], points[:,1], triangles, mask=mask)
# mesh = axes.plot_trisurf(points[:,0], points[:,1], triangles, points[:,2])
mesh = axes.plot_trisurf(t, points[:,2])
axes.plot_trisurf(points[:,0][:3], points[:,1][:3], points[:,1][:3], color='r')
pyplot.show()
