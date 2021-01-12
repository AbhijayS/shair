from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot, tri
import matplotlib.animation as animation
import numpy as np
import trimesh
import math
import struct

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
  return origin_point + local_units[0]*local_point[0] + local_units[1]*local_point[1] + local_units[2]*local_point[2]

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
    if ua>=0 and ua<=1 and ub>=0 and ub<=1:
      x = p1[0] + ua*(p2[0]-p1[0])
      y = p1[1] + ua*(p2[1]-p1[1])
      return (1, (x,y))
    return (0,)

def adjacent_triangle(tri, ends):
  # tri and ends contain point indices
  # returns adj triangle and its normal
  global triangles

  for p in tri:
    if not p in ends:
      third = p

  for i in range(len(triangles)):
    t = triangles[i]
    if (ends[0] in t) and (ends[1] in t) and (not third in t):
      n = normals[i]
      return (t,n)

# https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
def rodrigues_rotation(n1, n2, v):
  cross = np.cross(n1, n2)
  a = cross / np.linalg.norm(cross)
  sinphi = np.linalg.norm(np.cross(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2)))
  cosphi = np.dot(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2))
  return v*cosphi + np.cross(a, v)*sinphi + a*np.dot(a, v)*(1-cosphi)


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

# Create a new plot
fig = pyplot.figure()
axes = mplot3d.Axes3D(fig)

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
unit_vectors = np.array([u,v,w])

# initial starting location
# in reference to uvw units
location = np.array([-0.003,0.01,0])

# move origin to new location
origin = point_local_to_global(unit_vectors, origin, location)

# set location back to zero (origin)
location = np.array([0,0,0])

# edge1: v0 -> v1
edge1_p0 = transform_coordinates(unit_vectors, v0-origin)
edge1_p1 = transform_coordinates(unit_vectors, v1-v0) + edge1_p0
# edge2: v1 -> v2
edge2_p0 = transform_coordinates(unit_vectors, v1-origin)
edge2_p1 = transform_coordinates(unit_vectors, v2-v1) + edge2_p0
# edge3: v2 -> v0
edge3_p0 = transform_coordinates(unit_vectors, v2-origin)
edge3_p1 = transform_coordinates(unit_vectors, v0-v2) + edge3_p0

def animate(i):
  global triangle
  global vertices
  global axes
  global origin
  global points

  axes.clear()
  
  # draw location marker and unit vectors
  axes.scatter(*origin, color='r', s=100)
  axes.quiver(*origin, *u, color='r')
  axes.quiver(*origin, *v, color='g')
  # axes.quiver(*origin, *w, color='b')
  
  # four adjacent triangles
  four = np.array([
    vertices,
    adjacent_triangle(vertices, [vertices[0], vertices[1]])[0],
    adjacent_triangle(vertices, [vertices[1], vertices[2]])[0],
    adjacent_triangle(vertices, [vertices[2], vertices[0]])[0]
  ])
  t = tri.Triangulation(points[:,0], points[:,1], four)
  mesh = axes.plot_trisurf(t, points[:,2], color=(0,0,0,0), edgecolor='Gray')

def press(event):
  global location
  global edge1_p0
  global edge1_p1
  global edge2_p0
  global edge2_p1
  global edge3_p0
  global edge3_p1
  global unit_vectors

  move = 0.05

  if event.key == 'up':
    movement_vector = np.array([0,move,0])
  elif event.key == 'right':
    movement_vector = np.array([move,0,0])
  elif event.key == 'down':
    movement_vector = np.array([0,-move,0])
  elif event.key == 'left':
    movement_vector = np.array([-move,0,0])
  
  # calculate intersections
  i1 = intersection_point(location, location+movement_vector, edge1_p0, edge1_p1)
  i2 = intersection_point(location, location+movement_vector, edge2_p0, edge2_p1)
  i3 = intersection_point(location, location+movement_vector, edge3_p0, edge3_p1)

  while movement_vector != [0,0,0]:
    if i1[0]==1 and i2[0]!=1 and i3[0]!=1:
      new_tri, new_norm = adjacent_triangle(vertices, [0,1])
      new_units = np.array(list(map(lambda x: rodrigues_rotation(unit_vectors[2], new_norm, x), unit_vectors)))
      print(new_units)
      axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[0], color='r')
      axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[1], color='g')
      # reassign unit vector
      # move origin
      # update movement_vector
    elif i2[0]==1 and i1[0]!=1 and i3[0]!=1:
    elif i3[0]==1 and i1[0]!=1 and i2[0]!=1:
    elif i3[0]==0 and i2[0]==0 and i3[0]==0:
      movement_vector = 0
    else:
      print("uh oh")



  print(movement_vector)



fig.canvas.mpl_connect('key_press_event', press)
ani = animation.FuncAnimation(fig, animate, interval=1000/24)
pyplot.show()