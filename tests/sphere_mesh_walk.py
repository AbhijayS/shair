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
    if ua>0 and ua<1 and ub>0 and ub<1:
      x = p1[0] + ua*(p2[0]-p1[0])
      y = p1[1] + ua*(p2[1]-p1[1])
      return (1, (x,y))
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
      return (i,n)

# https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
def rodrigues_rotation(n1, n2, v):
  cross = np.cross(n1, n2)
  if not cross.any():
    return v
  a = cross / np.linalg.norm(cross)
  sinphi = np.linalg.norm(np.cross(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2)))
  cosphi = np.dot(n1/np.linalg.norm(n1), n2/np.linalg.norm(n2))
  return v*cosphi + np.cross(a, v)*sinphi + a*np.dot(a, v)*(1-cosphi)


with open('tests/sphere.stl', 'rb') as f:
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
location = np.array([-0.003,0.01,0.])

# move origin to new location
origin = point_local_to_global(unit_vectors, origin, location)

# set location back to zero (origin)
# always zero (relative to origin)
location = np.array([0.,0.,0.])
prev_origin = None
def animate(i):
  global triangles
  global triangle
  global vertices
  global axes
  global origin
  global points
  global prev_origin

  # if np.array_equal(prev_origin, origin):
  #   return

  # prev_origin = origin

  axes.clear()
  
  # draw location marker and unit vectors
  axes.scatter(*origin, color='r', s=100)
  axes.quiver(*origin, *u, color='r')
  axes.quiver(*origin, *v, color='g')
  axes.quiver(*origin, *w, color='b')
  
  # four adjacent triangles
  four = np.array([
    vertices,
    triangles[adjacent_triangle(vertices, [vertices[0], vertices[1]])[0]],
    triangles[adjacent_triangle(vertices, [vertices[1], vertices[2]])[0]],
    triangles[adjacent_triangle(vertices, [vertices[2], vertices[0]])[0]]
  ])
  t = tri.Triangulation(points[:,0], points[:,1], triangles)
  mesh = axes.plot_trisurf(t, points[:,2], color=(0,0,0,0), edgecolor='Gray')

def press(event):
  global location
  # global edge1_p0
  # global edge1_p1
  # global edge2_p0
  # global edge2_p1
  # global edge3_p0
  # global edge3_p1
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

  print(event.key)

  movement_vector = [0,0,0]

  if event.key == 'up':
    movement_vector = np.array([0.,move,0.])
  elif event.key == 'right':
    movement_vector = np.array([move,0.,0.])
  elif event.key == 'down':
    movement_vector = np.array([0.,-move,0.])
  elif event.key == 'left':
    movement_vector = np.array([-move,0.,0.])
  elif event.key == 'e':
    unit_vectors = np.matmul([
      [math.cos(rotate), -math.sin(rotate), 0],
      [math.sin(rotate), math.cos(rotate), 0],
      [0,0,1]
    ], unit_vectors)
    u,v,w = unit_vectors
    return
  elif event.key == 'r':
    unit_vectors = np.matmul([
      [math.cos(-rotate), -math.sin(-rotate), 0],
      [math.sin(-rotate), math.cos(-rotate), 0],
      [0,0,1]
    ], unit_vectors)
    u,v,w = unit_vectors
    return
  else:
    return

  previous_intersection_edge = None

  while not np.array_equal(movement_vector, [0,0,0]):
    # calculate intersections
    # edge1: v0 -> v1
    if not np.array_equal(previous_intersection_edge, [v1,v0]):
      edge1_p0 = transform_coordinates(unit_vectors, v0-origin)
      edge1_p1 = transform_coordinates(unit_vectors, v1-v0) + edge1_p0
      i1 = intersection_point(location, location+movement_vector, edge1_p0, edge1_p1)
    else:
      i1 = (0,)

    # edge2: v1 -> v2
    if not np.array_equal(previous_intersection_edge, [v2,v1]):
      edge2_p0 = transform_coordinates(unit_vectors, v1-origin)
      edge2_p1 = transform_coordinates(unit_vectors, v2-v1) + edge2_p0
      i2 = intersection_point(location, location+movement_vector, edge2_p0, edge2_p1)
    else:
      i2 = (0,)

    # edge3: v2 -> v0
    if not np.array_equal(previous_intersection_edge, [v0,v2]):
      edge3_p0 = transform_coordinates(unit_vectors, v2-origin)
      edge3_p1 = transform_coordinates(unit_vectors, v0-v2) + edge3_p0
      i3 = intersection_point(location, location+movement_vector, edge3_p0, edge3_p1)
    else:
      i3 = (0,)

    # print("mov:" + str(movement_vector))
    # print("tri: " + str(triangle))
    # print("prev:" + str(previous_intersection_edge))
    # print(i1,i2,i3)
    # print("")

    if i1[0]==1 and i2[0]!=1 and i3[0]!=1:
      previous_intersection_edge = [v0, v1]
      
      # update movement vector
      movement_vector = movement_vector-(i1[1]+(0,))

      # update origin point to intersection point
      origin = point_local_to_global(unit_vectors, origin, i1[1]+(0,))
      
      # calculate new triangle and vertices
      triangle, new_norm = adjacent_triangle(vertices, [vertices[0], vertices[1]])
      vertices = triangles[triangle]
      v0,v1,v2 = map(lambda v: points[v], vertices)

      # calculate new unit vectors on new triangle
      unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(unit_vectors[2], new_norm, x), unit_vectors)))
      u,v,w = unit_vectors

      # axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[0], color='r')
      # axes.quiver(*point_local_to_global(unit_vectors, origin, i1[1]+(0,)), *new_units[1], color='g')

    elif i2[0]==1 and i1[0]!=1 and i3[0]!=1:
      previous_intersection_edge = [v1, v2]

      # update movement vector
      movement_vector = movement_vector-(i2[1]+(0,))

      # update origin point to intersection point
      origin = point_local_to_global(unit_vectors, origin, i2[1]+(0,))
      
      # calculate new triangle and vertices
      triangle, new_norm = adjacent_triangle(vertices, [vertices[1], vertices[2]])
      vertices = triangles[triangle]
      v0,v1,v2 = map(lambda v: points[v], vertices)

      # calculate new unit vectors on new triangle
      unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(unit_vectors[2], new_norm, x), unit_vectors)))
      u,v,w = unit_vectors

    elif i3[0]==1 and i1[0]!=1 and i2[0]!=1:
      previous_intersection_edge = [v2, v0]

      # update movement vector
      movement_vector = movement_vector-(i3[1]+(0,))

      # update origin point to intersection point
      origin = point_local_to_global(unit_vectors, origin, i3[1]+(0,))
      
      # calculate new triangle and vertices
      triangle, new_norm = adjacent_triangle(vertices, [vertices[2], vertices[0]])
      vertices = triangles[triangle]
      v0,v1,v2 = map(lambda v: points[v], vertices)

      # calculate new unit vectors on new triangle
      unit_vectors = np.array(list(map(lambda x: rodrigues_rotation(unit_vectors[2], new_norm, x), unit_vectors)))
      u,v,w = unit_vectors

    elif i3[0]==0 and i2[0]==0 and i3[0]==0:
      previous_intersection_edge = None
      origin = point_local_to_global(unit_vectors, origin, movement_vector)
      movement_vector = [0,0,0]

    else:
      print("uh oh")
      break

fig.canvas.mpl_connect('key_press_event', press)
ani = animation.FuncAnimation(fig, animate, interval=1000/2)
pyplot.show()