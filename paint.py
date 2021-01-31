import numpy as np
import struct
import sys
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib import tri
import matplotlib.animation as animation
from matplotlib.colors import LinearSegmentedColormap

#Definitions
fig = plt.figure()
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

#Identifies triangles on top and adds certain value to them 
# def paint(filename):
    
#     triangledata = read_stl(filename)[2]
#     points = read_stl(filename)[0]
#     triangles = read_stl(filename)[2]

#     # Need to mask out certain triangles and add a value between 0 and 1 in np.array(triangles) to them
#     #individualtriangle = axes.plot_trisurf(t, points[:,2], color=(0,0,0,0), edgecolor='Gray')
#     #To insert values into ndarray must convert to list
#     triangledata = triangledata.tolist()
#     triangledata[2].append(0.256)
    
#     return points

"""ANIMATION METHOD"""
def animate(i):
    
    points = read_stl('JUST HEAD 1000.stl')[0]
    triangles = read_stl('JUST HEAD 1000.stl')[2]
    axes.clear()

    

    # For each triangle, compute center, divide by z total height, feed to cmap(gray), append to list
    mycolors = []
    for triangle in triangles:
        Oz = (points[triangle[0]][2] + points[triangle[1]][2] + points[triangle[2]][2])/3
        Oz = Oz/600    #max(points[triangles[0]][2], points[triangles[1]][2], points[triangles[2]][2])
        Ozlist = [Oz, Oz, Oz, 1]
        mycolors.append(Ozlist)

    # print(points[triangles[0][0]][2])
    # print(points[triangles[0][1]][2])
    # print(points[triangles[0][2]][2])
    # print((points[triangles[0][0]][2] + points[triangles[1][1]][2] + points[triangles[2][2]][2])/3)
    # np.set_printoptions(threshold=sys.maxsize)
    # mycolors = np.array(mycolors)
    # print(mycolors[0])
    
    cmap_name = 'my_list'
    cm = LinearSegmentedColormap.from_list(cmap_name, mycolors)

    #my_cmap = plt.get_cmap('hot')
    t = tri.Triangulation(points[:,0], points[:,1], triangles)
    axes.plot_trisurf(t, points[:,2], cmap = cm, edgecolor='Black')

    axes.set_zlabel('z-axis')

    


    #t2 = tri.Triangulation(points[:,0], points[:,1], triangles[1:2])
    #axes.plot_trisurf(t2, points[:,2], color=(0.8,0.8,0.8,1), edgecolor='Black')

    #t = tri.Triangulation(points[:,0], points[:,1], triangles)
    #mesh = axes.plot_trisurf(t, points[:,2], cmap = my_cmap, edgecolor='Black')


# ani = animation.FuncAnimation(fig, animate, interval=1000/24)

points = read_stl('JUST HEAD 1000.stl')[0]
triangles = read_stl('JUST HEAD 1000.stl')[2]
axes.clear()



# For each triangle, compute center, divide by z total height, feed to cmap(gray), append to list
mycolors = []
for triangle in triangles:
    Oz = (points[triangle[0]][2] + points[triangle[1]][2] + points[triangle[2]][2])/3
    Oz = Oz/600    #max(points[triangles[0]][2], points[triangles[1]][2], points[triangles[2]][2])
    Ozlist = [Oz, Oz, Oz, 1]
    mycolors.append(Ozlist)
    p1 = points[triangle[0]]
    p2 = points[triangle[1]]
    p3 = points[triangle[2]]
    x = [p1[0], p2[0], p3[0]]
    y = [p1[1], p2[1], p3[1]]
    z = [p1[2], p2[2], p3[2]]
    # print(x)
    # print(y)
    # print(z)
    # print(triangle)
    # t = tri.Triangulation(x, y, np.array([triangle]))
    axes.plot_trisurf(x,y, [[0,1,2]], z, color=Ozlist, edgecolor='Black')
    

# print(points[triangles[0][0]][2])
# print(points[triangles[0][1]][2])
# print(points[triangles[0][2]][2])
# print((points[triangles[0][0]][2] + points[triangles[1][1]][2] + points[triangles[2][2]][2])/3)
# np.set_printoptions(threshold=sys.maxsize)
# mycolors = np.array(mycolors)
# print(mycolors[0])

# cmap_name = 'my_list'
# cm = LinearSegmentedColormap.from_list(cmap_name, mycolors)

# #my_cmap = plt.get_cmap('hot')
# t = tri.Triangulation(points[:,0], points[:,1], triangles)
# axes.plot_trisurf(t, points[:,2], cmap = cm, edgecolor='Black')

axes.set_zlabel('z-axis')

plt.show()

#Write to triangledata.txt to see triangles
# np.set_printoptions(threshold=sys.maxsize)
# triangledata = paint('JUST HEAD 1000.stl')
# f = open("triangledata.txt", "w")
# f.write(str(triangledata))





