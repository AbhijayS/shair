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


def point_local_to_global(local_units, origin_point, local_point):
  return origin_point + local_units*local_point

def vector_local_to_global(local_units, local_vector):
  return np.sum(local_units*local_vector, axis=0)

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

vertices = triangles[0]
v0 = points[vertices[0]]
v1 = points[vertices[1]]
v2 = points[vertices[2]]

# triangle coordinates origin point
# always the first vertex in the triangle
origin = v0

# triangle coordinates unit vectors uvw
w = np.array(normals[0])
v = (v1-v0) / math.hypot(*(v1-v0))
u = np.cross(v, w) / math.hypot(*np.cross(v, w))
unit_vectors = np.array([u,v,w])

# relative to triangle coordinates (uvw)
current_location = [0,0.01,0]

# planar movement using uv units (w is ignored)
movement = np.array([current_location,[1,0.01,0]])

# axes.quiver(*point_local_to_global(unit_vectors, origin, current_location), *vector_local_to_global(unit_vectors, movement[1]-movement[0]), color='black')

# edge1: v0 -> v1
edge1_p0 = transform_coordinates(unit_vectors, v0-origin)
edge1_p1 = transform_coordinates(unit_vectors, v1-v0) + edge1_p0
# edge2: v1 -> v2
edge2_p0 = transform_coordinates(unit_vectors, v1-origin)
edge2_p1 = transform_coordinates(unit_vectors, v2-v1) + edge2_p0
# edge3: v2 -> v0
edge3_p0 = transform_coordinates(unit_vectors, v2-origin)
edge3_p1 = transform_coordinates(unit_vectors, v0-v2) + edge3_p0

i1 = intersection_point(movement[0], movement[1], edge1_p0, edge1_p1)[1]
i2 = intersection_point(movement[0], movement[1], edge2_p0, edge2_p1)[1]
i3 = intersection_point(movement[0], movement[1], edge3_p0, edge3_p1)
print(i1)
print(i2)
print(i3)

# i1_g = origin + u*i1[0] + v*i1[1]
# i2_g = origin + u*i2[0] + v*i2[1]
# axes.scatter(i1_g[0], i1_g[1], i1_g[2], s=100, c='r')
# axes.scatter(i2_g[0], i2_g[1], i2_g[2], s=100, c='b')

axes.quiver(*origin, *u, color='r')
axes.quiver(*origin, *v, color='g')
axes.quiver(*origin, *w, color='b')
# axes.quiver(*current_location, *())

mask = [False]*len(triangles)
mask[0] = True
t = tri.Triangulation(points[:,0], points[:,1], triangles, mask=mask)
mesh = axes.plot_trisurf(t, points[:,2], color=(0,0,0,0), edgecolor='Gray')
pyplot.show()
