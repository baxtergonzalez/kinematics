import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
from scipy.spatial.transform import Rotation as R


fig = plt.figure()
ax = plt.axes(projection='3d')
origin = np.array([0,0,0])

def rotationMatrix(rAng):
    '''
    -takes in euler angles, spit out rotation matrix
    '''
    r = R.from_euler('xyz', rAng, degrees = True)
    rotation =np.array(r.as_matrix())
    return rotation

def drawVector(start, end, col='b'):
    ax.quiver(start[0], start[1], start[2], end[0], end[1], end[2], color = col)

rotationAngles = [45,15,180]

vector1 = np.array([20,0,0])
vector2 = np.array([0,20,0])
vector3 = np.array([0,0,20])

Rvector1 = np.dot(rotationMatrix(rotationAngles), vector1)
Rvector2 = np.dot(rotationMatrix(rotationAngles), vector2)
Rvector3 = np.dot(rotationMatrix(rotationAngles), vector3)

drawVector(origin, vector1,'r')
drawVector(origin, vector2,'y')
drawVector(origin, vector3)

drawVector(origin, Rvector1, 'g')
drawVector(origin, Rvector2, 'g')
drawVector(origin, Rvector3, 'g')


ax.set_xlim([-20,20])
ax.set_ylim([-20,20])
ax.set_zlim([-0,30])

ax.set_xlabel('X coord')
ax.set_ylabel('Y coord')
ax.set_zlabel('Z coord')

plt.show()