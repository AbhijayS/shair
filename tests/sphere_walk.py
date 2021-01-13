import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
from math import sin, cos, asin, atan2, radians, pi

# credits: https://www.movable-type.co.uk/scripts/latlong.html
def to_n_vect(lat, lon, radius):
    x = radius*cos(lat)*cos(lon)
    y = radius*cos(lat)*sin(lon)
    z = radius*sin(lat)
    return (x,y,z)

def bearing_radians(lat1, lon1, lat2, lon2):
    y = sin(lon2-lon1) * cos(lat2)
    x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1)
    θ = atan2(y, x)
    return θ
    # return this for degrees
    # return (θ*180/pi + 360) % 360 # range from 0 to 360

def destination_point(lat, lon, bearing, dist, radius):
    lat2 = asin(sin(lat)*cos(dist/radius)+cos(lat)*sin(dist/radius)*cos(bearing))
    lon2 = lon + atan2(sin(bearing)*sin(dist/radius)*cos(lat), cos(dist/radius)-sin(lat)*sin(lat2))
    final_bearing = bearing_radians(lat2, lon2, lat, lon)
    bearing2 = final_bearing+pi
    return (lat2, lon2, bearing2)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# calculate final point
# dp = destination_point(p[0], p[1], p[2], 3, r)
# print('second ' + str(dp))

# draw final point
# dpnvect = to_n_vect(dp[0], dp[1], r)
# ax.scatter(dpnvect[0], dpnvect[1], dpnvect[2], c='g', s = 100)

# point: lat, lon, bearing
p = (0, 0, 0)
move = 1
# keyboard interaction
def press(event):
    global p
    global p_plot
    global ax

    print('press', event.key)
    if event.key == 'up':
        p = destination_point(p[0], p[1], p[2], move, r)
    elif event.key == 'right':
        temp = destination_point(p[0], p[1], p[2]+pi/2, move, r)
        p = (temp[0], temp[1], temp[2]-pi/2)
    elif event.key == 'left':
        temp = destination_point(p[0], p[1], p[2]-pi/2, move, r)
        p = (temp[0], temp[1], temp[2]+pi/2)
    elif event.key == 'down':
        temp = destination_point(p[0], p[1], p[2]+pi, move, r)
        p = p = (temp[0], temp[1], temp[2]-pi)
    print(p)


def animate(i):
    global ax
    global p
    global r

    ax.clear()
    # draw sphere
    r = 10
    lat = np.linspace(-np.pi/2, np.pi/2, 20) # latitude
    lon = np.linspace(0, 2*np.pi, 20) # longtitude
    x = r * np.outer(np.cos(lat), np.cos(lon))
    y = r * np.outer(np.cos(lat), np.sin(lon))
    z = r * np.outer(np.sin(lat), np.ones(np.size(lat)))
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.plot_wireframe(x, y, z, color='b')

    # draw point
    # order: lat, lon, bearing
    pnvect = to_n_vect(p[0], p[1], r)
    p_plot = ax.scatter(pnvect[0], pnvect[1], pnvect[2], c='r', s = 100)

fig.canvas.mpl_connect('key_press_event', press)
ani = animation.FuncAnimation(fig, animate, interval=1000/24)
# ax.set_aspect(auto)
plt.show()
