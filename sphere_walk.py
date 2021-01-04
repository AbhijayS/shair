import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from math import sin, cos, asin, atan2, radians

# credits: https://www.movable-type.co.uk/scripts/latlong.html
def to_n_vect(lat, lon, radius):
    x = radius*cos(lat)*cos(lon)
    y = radius*cos(lat)*sin(lon)
    z = radius*sin(lat)
    return (x,y,z)

def destinationPoint(lat, lon, bearing, dist, radius):
    lat2 = asin(sin(lat)*cos(dist/radius)+cos(lat)*sin(dist/radius)*cos(bearing))
    lon2 = lon + atan2(sin(bearing)*sin(dist/radius)*cos(lat), cos(dist/radius)-sin(lat)*sin(lat2))
    bearing2 = (bearing+180)%360
    return (lat2, lon2, bearing2)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# draw sphere
r = 10
lat = np.linspace(-np.pi/2, np.pi/2, 20) # latitude
lon = np.linspace(0, 2*np.pi, 20) # longtitude
x = r * np.outer(np.cos(lat), np.cos(lon))
y = r * np.outer(np.cos(lat), np.sin(lon))
z = r * np.outer(np.sin(lat), np.ones(np.size(lat)))

# draw start point
plat = 0
plon = 0
pnvect = to_n_vect(plat, plon, r)
ax.scatter(pnvect[0], pnvect[1], pnvect[2], c='r', s = 100)


# calculate final point
dp = destinationPoint(plat, plon, radians(30), 10, r)

# draw final point
dpnvect = to_n_vect(dp[0], dp[1], r)
ax.scatter(dpnvect[0], dpnvect[1], dpnvect[2], c='r', s = 100)


ax.plot_wireframe(x, y, z, color='b')

plt.show()
