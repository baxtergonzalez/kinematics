import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
#import plotly.graph_objects as go

def drawVector(start, end, col):
    ax.quiver(start[0], start[1], start[2], end[0], end[1], end[2], color=col)

def cyl2Cart(length, thetaXY, deltaZ):
    """
    -converts cylindrically expressed vector into set of displacements in x, y, and z in cartesian space
    """
    dx=length*np.cos(thetaXY)
    dy=length*np.sin(thetaXY)
    dz=deltaZ
    
    return dx, dy, dz
def rotationMatrix(rAng):
    rX,rY,rZ = np.radians(rAng[0]), np.radians(rAng[1]), np.radians(rAng[2])
    # rotation = np.array([[np.cos(rX)*np.cos(rY), np.cos(rX)*np.sin(rY)*np.sin(rZ)-np.sin(rX)*np.cos(rZ), np.cos(rX)*np.sin(rY)*np.cos(rZ)+np.sin(rX)*np.sin(rY)],
    #                     [np.sin(rX)*np.cos(rY), np.sin(rX)*np.sin(rY)*np.sin(rZ)+np.cos(rX)*np.cos(rZ), np.sin(rX)*np.sin(rY)*np.cos(rZ)-np.cos(rX)*np.sin(rZ)],
    #                     [-np.sin(rZ), np.cos(rY)*np.sin(rZ), np.cos(rY)*np.cos(rZ)]])
    #return rotation
#-------------------------------------------------
#--------------design variables-------------------
#-------------------------------------------------

#----------base location vectors------------
aLength = 10 #length of a vectors in mm (constant)
a1=np.array(cyl2Cart(aLength,np.radians((360/3)*1),0))
a2=np.array(cyl2Cart(aLength,np.radians((360/3)*2),0))
a3=np.array(cyl2Cart(aLength,np.radians((360/3)*3),0))

#----------linkage 3 vectors------------
dLength = 10 #length of c vectors in mm (constant)
d1 = np.array(cyl2Cart(-dLength,np.radians((360/3)*1), 0))
d2 = np.array(cyl2Cart(-dLength,np.radians((360/3)*2), 0))
d3 = np.array(cyl2Cart(-dLength,np.radians((360/3)*3), 0))


#-------------------------------------------------
#-------------adjustable variables----------------
#-------------------------------------------------

platformRotations = [20,20,20]
#----------rotation matrix-------------
R=np.array(rotationMatrix(platformRotations))
position = np.array([0,0,15])

#-------------------------------------------------
#-----------------output--------------------------
#-------------------------------------------------

#----------linkage 2 vectors------------
bLength = 15 #length of b vectors in mm (constant)
b1 = np.array(cyl2Cart(bLength,np.radians((360/3)*1), 15))
b2 = np.array(cyl2Cart(bLength,np.radians((360/3)*2), 15))
b3 = np.array(cyl2Cart(bLength,np.radians((360/3)*3), 15))

#----------linkage 3 vectors------------
cLength = 15 #length of c vectors in mm (constant)
c1 = np.array(cyl2Cart(-cLength,np.radians((360/3)*1), 20))
c2 = np.array(cyl2Cart(-cLength,np.radians((360/3)*2), 20))
c3 = np.array(cyl2Cart(-cLength,np.radians((360/3)*3), 20))

#-------------------------------------------------
#----------------display--------------------------
#-------------------------------------------------
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim([-20,20])
ax.set_ylim([-20,20])
ax.set_zlim([-0,30])

ax.set_xlabel('X coord')
ax.set_ylabel('Y coord')
ax.set_zlabel('Z coord')

origin = [0,0,0]

drawVector(origin,a1,'g')
drawVector(origin,a2,'g')
drawVector(origin,a3,'g')

drawVector(a1,b1,'b')
drawVector(a2,b2,'b')
drawVector(a3,b3,'b')

drawVector((b1+a1),c1,'y')
drawVector((b2+a2),c2,'y')
drawVector((b3+a3),c3,'y')

drawVector((b1+a1+c1),d1,'r')
drawVector((b2+a2+c2),d2,'r')
drawVector((b3+a3+c3),d3,'r')

plt.show()
