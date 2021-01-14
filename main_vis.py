import csv
from mpl_toolkits import mplot3d
from matplotlib import pyplot, tri
import matplotlib.animation as animation
import numpy as np
import struct

# Create a new plot
fig = pyplot.figure()
axes = mplot3d.Axes3D(fig)

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

points, normals, triangles = read_stl('tests/sphere.stl')
prev = None

def animate(i):
    global axes
    global fig
    global points
    global normals
    global triangles
    global prev


    with open('clipper.csv', 'r') as f:
        reader = csv.DictReader(f)
        line = {}
        for r in reader:
            line = r 
        line['units'] = eval(line['units'].replace('(','').replace(')','').replace('array',''))
        line['pose3d'] = eval(line['pose3d'].replace('(','').replace(')','').replace('array',''))

        if prev == line['pose3d']:
            return
            
        axes.clear()
        
        # draw location marker and unit vectors
        axes.scatter(*line['pose3d'], color='r', s=100)
        axes.quiver(*line['pose3d'], *line['units'][0], color='r')
        axes.quiver(*line['pose3d'], *line['units'][1], color='g')
        axes.quiver(*line['pose3d'], *line['units'][2], color='b')
        prev = line['pose3d']

    # replot mesh
    # four adjacent triangles
    # four = np.array([
    #     vertices,
    #     triangles[adjacent_triangle(vertices, [vertices[0], vertices[1]])[0]],
    #     triangles[adjacent_triangle(vertices, [vertices[1], vertices[2]])[0]],
    #     triangles[adjacent_triangle(vertices, [vertices[2], vertices[0]])[0]]
    # ])
    t = tri.Triangulation(points[:,0], points[:,1], triangles)
    axes.plot_trisurf(t, points[:,2], color=(0,0,0,0), edgecolor='Gray')

ani = animation.FuncAnimation(fig, animate, interval=1000/10)
pyplot.show()