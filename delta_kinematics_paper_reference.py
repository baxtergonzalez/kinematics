#referencing https://hypertriangle.com/~alex/delta-robot-tutorial/
# and https://www.ohio.edu/mechanical-faculty/williams/html/PDF/DeltaKin.pdf 

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
from numpy.core.shape_base import vstack
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
        -input determines whether or not the vector is calculated as a cartesian or cylindrical
    -outputs a numpy array in cartesian form for 3D vector
    -if cartesian, straight up just turn array into numpy array
        -can possibly take a numpy array? unclear tbh, i am too lazy to test
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
    -angle input is in degrees
    '''
    r = R.from_euler('xyz', rAng, degrees = True)
    rotation =np.array(r.as_matrix())
    return rotation

def drawVector(start, end, col='b'):
    '''
    -displays the vector from a starting point to ending point
    -default color is blue
    '''
    ax.quiver(start[0], start[1], start[2], end[0], end[1], end[2], color=col)

def drawSphere(origin, radius, draw = False, col = 'r'):
    """
    -draw a sphere around input position of radius cLength + dLength(linkage + platform length)
    -represents all possible positions of platform
    -intersection of this sphere with circles drawn around the start of linkage b is the endpoint of where the b linkage should be
    """
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = radius* np.cos(u)*np.sin(v) + origin[0]
    y = radius* np.sin(u)*np.sin(v)+ origin[1]
    z = radius* np.cos(v)+ origin[2]
    if draw:
        ax.plot_wireframe(x, y, z, color=col)

    coords = np.array([x,y,z])
    return coords

def drawCircle(origin, radius, phi, bMin = 0, bMax = 2*np.pi, resolution = 50, draw = False):
    """
    -draw a circle of radius bLength centered at the start of the b joint
    -the circle is constrained to be co-planer with the b linkage
    -this circle represents all of the possible positions for b
    -can also restrict the circle into a segment to represent the range of motors
    """
    
    theta = np.linspace(bMin, bMax, resolution)
    x = np.array(radius*np.cos(theta)*np.cos(phi) + origin[0])
    y = np.array(radius*np.cos(theta)*np.sin(phi) + origin[1])
    z = np.array(radius*np.sin(theta) + origin[2])
    if draw:
        ax.plot(x,y,z)
    coords = vstack(np.meshgrid(x,y,z)).reshape(3,-1).T
    #coords = np.meshgrid(x,y,z)
    
    return coords

def findDistance3D(point1, point2):
    """ 
    -returns the distance between two provided coordinates in 3D cartesian space
    """
    distance = np.sqrt(((point1[0]-point2[0])**2)+((point1[1]-point2[1])**2)+((point1[2]-point2[2])**2)) 
    return distance

def findIntersectionPoint(bLineSet, origin, radius, tolerance = 30, draw = True):
    """
    -(blineSet): provide set of points to compare
    -(origin/radius): provide origin of a sphere as well as it's radius
    -(tolerance): provide a minimum tolerance that your points must be within from the surface of the sphere
    
    This function checks the distance of all of the points of a set from the center of a sphere.
    
    The goal is to return the point, if there is one, within the set of points provided that intersects with 
    the surface of a sphere
        If the distance from the center is less than the radius, it is not on the surface
        If the distance from the center is greater than the radius, it is not on the surface
    
    """
    threshold = radius/tolerance
           
    for point in bLineSet:
        if (findDistance3D(point,origin)-radius)<threshold:
            intersectionPoint = point
            if draw:
                drawSphere(intersectionPoint, 1, col='b')
            return intersectionPoint
        else:
            intersectionPoint = np.array([0,0,0])

# position vector is the input position vector of the center of end effector
# rotation is the rotation matrix for the orientation of the end effector (input in degrees)
# origin is constant. always (0,0,0)

position = makeVect([5,10,25])
rotation = rotationMatrix([0,0,0])
origin = makeVect([0,0,0])
cLength = 25
bLength = 10

# a vectors are first grounded linkage from origin

aLength = 15
a1 = makeVect([aLength, 0, 0], cyl = True)
a2 = makeVect([aLength, (360/3)*1, 0], cyl = True)
a3 = makeVect([aLength, (360/3)*-1, 0], cyl = True)

# d vectors are platform vectors going from position to connect to legs

dLength = 5
d1 = np.dot(makeVect([dLength, 0, 0], cyl = True), rotation)
d2 = np.dot(makeVect([dLength, (360/3)*1, 0], cyl = True), rotation)
d3 = np.dot(makeVect([dLength, (360/3)*-1, 0], cyl = True), rotation)

# draw static design vectors a 1-3 and d 1-3
drawVector(origin, a1)
drawVector(origin, a2)
drawVector(origin, a3)

drawVector(position, d1, 'g')
drawVector(position, d2, 'g')
drawVector(position, d3, 'g')


#draw vectors b 1-3, the vectors that 
b1 = findIntersectionPoint(drawCircle(a1, bLength, 0, np.pi/4, 3*np.pi/4, draw = True), position+d1,cLength, tolerance = 300) - a1
drawVector(a1, b1)
b2 = findIntersectionPoint(drawCircle(a2, bLength, 2*np.pi/3,  np.pi/4, 3*np.pi/4, draw = True), position+d2, cLength, tolerance = 300)-a2
drawVector(a2, b2)
b3 = findIntersectionPoint(drawCircle(a3, bLength,  2*np.pi/-3, np.pi/4, 3*np.pi/4, draw = True), position+d3, cLength, tolerance = 300)-a3
drawVector(a3, b3)

drawVector(b1+a1, position+d1-a1-b1, col = 'r')
drawVector(b2+a2, position+d2-a2-b2, col = 'r')
drawVector(b3+a3, position+d3-a3-b3, col = 'r')

# display plot
# !!ALWAYS AT END OF SCRIPT!!
plt.show()