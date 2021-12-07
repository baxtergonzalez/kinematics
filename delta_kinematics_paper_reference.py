#referencing https://hypertriangle.com/~alex/delta-robot-tutorial/

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
from scipy.spatial.transform import Rotation as R


#setup for the plotting functions

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim([-20,20])
ax.set_ylim([-20,20])
ax.set_zlim([-0,30])

ax.set_xlabel('X coord')
ax.set_ylabel('Y coord')
ax.set_zlabel('Z coord')


def makeVect(input, cyl=False):
    """
    -takes in input either with x y an z for cartesian input or length, thetaXY, and deltaZ if cylingrical
    -outputs a numpy array in cartesian form for 3D vector
    """
    if not cyl:
        x=input[0]
        y=input[1]
        z=input[2]

        vector = np.array([input[0],input[1],input[2]])
    elif cyl:
        length = input[0]
        thetaXY = np.radians(input[1])
        deltaZ = input[2]

        dx=length*np.cos(thetaXY)
        dy=length*np.sin(thetaXY)
        dz=deltaZ 

        vector = np.array([dx,dy,dz])
    return vector

def rotationMatrix(rAng):
    '''
    -takes in euler angles, spit out rotation matrix

    '''
    r = R.from_euler('xyz', rAng, degrees = True)
    rotation =np.array(r.as_matrix())
    return rotation

def drawVector(start, end, col='b'):
    ax.quiver(start[0], start[1], start[2], end[0], end[1], end[2], color=col)

# position vector is the input position vector of the center of end effector
# rotation is the rotation matrix for the orientation of the end effector (input in degrees)
# origin is constant. always (0,0,0)

position = makeVect([0,0,15])
rotation = rotationMatrix([0,0,0])
origin = makeVect([0,0,0])

# a vectors are first grounded linkage from origin

aLength = 10
a1 = makeVect([aLength, (360/3)*1, 0], cyl = True)
a2 = makeVect([aLength, (360/3)*2, 0], cyl = True)
a3 = makeVect([aLength, (360/3)*3, 0], cyl = True)

drawVector(origin, a1)
drawVector(origin, a2)
drawVector(origin, a3)

# display plot
# !!ALWAYS AT END OF SCRIPT!!
plt.show()