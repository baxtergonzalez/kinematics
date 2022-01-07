'''
Written by Baxter Gonzalez and Daniel Wanegar

This program simulates, renders, and calculates the required angles of a delta-configuration parallel manipulator using two approaches.
The inverse-kinematics are solved with an analytical solution, returning 0 percent error, and a geometric solution returning a percent error as a function of the input resolution.

referencing https://hypertriangle.com/~alex/delta-robot-tutorial/
and https://www.ohio.edu/mechanical-faculty/williams/html/PDF/DeltaKin.pdf

'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
from numpy.core.shape_base import vstack
from scipy.spatial.transform import Rotation as R
import math

#setup for the plotting functions
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim([-20,20])
ax.set_ylim([-20,20])
ax.set_zlim([-0,30])

ax.set_xlabel('X coord')
ax.set_ylabel('Y coord')
ax.set_zlabel('Z coord')

class Linkage:
    def __init__(self, length, startPoint, cartesianDisplacement, phi):
        '''Linkage is a link in the manipulator. A series of linkages must connect to 'ground' at some point, a static reference frame. This point will be the origin for this case.

        Keyword arguments:
        length -- the magnitude of the linkage vector
        startPoint -- the position vector of the start of the linkage
        cartesianDisplacement -- the linkage vector in x,y, and z components
        phi -- the rotation of the linkage about the z axis passing through the origin
        '''
        self.length = length
        self.startPoint = np.array([startPoint[0], startPoint[1], startPoint[2]])
        self.cartDisp = np.array([cartesianDisplacement[0], cartesianDisplacement[1], cartesianDisplacement[2]])
        self.phi = phi
        self.endPoint = self.startPoint + self.cartDisp

        #express displacement in cylindrical coordinates
        self.cylDisplacement = np.array([length, np.radians(phi), cartesianDisplacement[2]])
    def drawLink(self, col='b'):
        '''
        Renders the linkage using matplotlib mplot3d
        '''
        ax.quiver(self.startPoint[0], self.startPoint[1], self.startPoint[2], self.endPoint[0], self.endPoint[1], self.endPoint[2], color = col)
    def findSquareDist(self, point1, point2):
        '''
        --returns the distance between two provided coordinates in 3D cartesian space, remove requirement for sqrt in each call
        -based on concept:
            -d = root(dx^2 + dy^2 + dz^2)
            -d^2 = dx^2 + dy^2 + dz^2
            -if a = b then a^2 = b^2
        -(essentially squaring both sides of the equation)
        '''
        squareDistance = (point1[0]-point2[0])**2+((point1[1]-point2[1])**2)+((point1[2]-point2[2])**2)
        return squareDistance

class UpperArm(Linkage):
    def __init__(self, length, startPoint, phi):
        super().__init__(length=length, startPoint=startPoint, cartesianDisplacement = [0,0,0], phi = phi)

    def drawSphere(self, col = 'r'):
        radius = self.length
        origin = self.endPoint

        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = radius* np.cos(u)*np.sin(v) + origin[0]
        y = radius* np.sin(u)*np.sin(v)+ origin[1]
        z = radius* np.cos(v)+ origin[2]

        ax.plot_wireframe(x, y, z, color=col)

    def verifyLength(self, inputLength, threshold = 1):
        if abs(inputLength - self.length) < threshold:
            return True
        else:
            return False

class ServoArm(Linkage):
    def __init__(self, length, startPoint, phi, minAngle, maxAngle, tolerance = 0.5, thetaTolerance = np.radians(1)):
        #


            #NOTE: need to overload the init function to not take a cartDisplacement vector as parameter



        #
        super().__init__(length=length, startPoint=startPoint, cartesianDisplacement=[0,0,0], phi=phi)
        self.minAngle = np.radians(minAngle)
        self.maxAngle = np.radians(maxAngle)
        self.tolerance = tolerance
        self.thetaTolerance = thetaTolerance
        self.makeArc()
        self.geometricSolve()
        self.analyticalSolve
    def makeArc(self, resolution):
        origin = self.startPoint
        radius = self.length
        phi = self.phi

        self.thetas = np.linspace(self.minAngle, self.maxAngle, resolution)
        theta = self.thetas

        x = np.array(radius*np.cos(theta)*np.cos(phi) + origin[0])
        y = np.array(radius*np.cos(theta)*np.sin(phi) + origin[1])
        z = np.array(radius*np.sin(theta) + origin[2])

        self.possiblePoints = np.array([x, y, z]).T
    def makePointOnArc(self, theta):
        origin = self.startPoint
        radius = self.length
        phi = self.phi

        x = radius*np.cos(theta)*np.cos(phi) + origin[0]
        y = radius*np.cos(theta)*np.sin(phi) + origin[1]
        z = radius*np.sin(theta) + origin[2]

        return np.array([x,y,z])

    def bisectMethod(self, link):
        thetaRight = self.maxAngle
        thetaLeft = self.minAngle

        deltaTheta = (thetaRight - thetaLeft)

        pointRight = self.makePointOnArc(thetaRight)
        pointLeft = self.makePointOnArc(thetaLeft)


        distance2Right = self.findSquareDist(pointRight, link.endPoint)
        distance2Left = self.findSquareDist(pointLeft, link.endPoint)

        errorRight = distance2Right - link.length**2
        errorLeft = distance2Left - link.length**2

        if abs(errorLeft) < self.tolerance:
            return thetaLeft
        if errorLeft*errorRight > 0:
            while errorLeft*errorRight > 0 and deltaTheta > self.thetaTolerance:
                thetaMid = (thetaRight - thetaLeft)/2
                deltaTheta /= 2
                pointMid = self.makePointOnArc(thetaMid)
                distance2Mid = self.findSquareDist(pointMid)
                errorMid = distance2Mid - link.length**2
                if abs(errorLeft)>abs(errorRight):
                    thetaLeft = thetaMid
                    pointLeft = pointMid
                    errorLeft = errorMid
                else:
                    thetaRight = thetaMid
                    pointRight = pointMid
                    errorRight = errorMid
        if deltaTheta < self.thetaTolerance:
            if abs(errorLeft)>abs(errorRight):
                self.theta = thetaRight
            else:
                self.theta = thetaLeft
            return "Theta tolerance reached, returning closest theta."

        while min(abs(errorLeft), abs(errorRight)) > self.tolerance and deltaTheta > self.thetaTolerance:
            thetaMid = (thetaRight - thetaLeft)/2
            deltaTheta /= 2
            pointMid = self.makePointOnArc(thetaMid)
            distance2Mid = self.findSquareDist(pointMid)
            errorMid = distance2Mid - link.length**2

            if errorLeft*errorMid > 0:
                thetaLeft = thetaMid
                pointLeft = pointMid
                errorLeft = errorMid
            else:
                thetaRight = thetaMid
                pointRight = pointMid
                errorRight = errorMid

        self.theta = thetaMid
            #TODO: Find point such that errorMid*errorLeft < 0 or bisectionDelta < thetaThreshold (minimum amount we can wiggle per bisection)
            #if minimum bisectionDelta reached, return theta and an error message
        
        #TODO: bisect until zero


        
        



    def drawArc(self):
        ax.plot(self.possiblePoints[0], self.possiblePoints[1], self.possiblePoints[2])

    def geometricSolve(self, link):
        '''Solves for the orientation of the Servo Arm linkage using a geometric method.

            Keyword Arguments:

            link -- an UpperArm object with attributes:
                                length -- the length of the linkage
                                endPoint -- the end point of the linkage
            
            Verbose Description:

            Finds the intersection of the arc centered on the starting point of the servo Arm of radius
            servo arm length, with the sphere centered on the connection point of the upper arm to the
            platform with radius upper arm length.

            The arc represents all possible positions of the end point of the servo arm, while the sphere
            (more specifically the surface of the sphere) represents all of the possible positions of the
            starting point of the upper arm. When these two points are equal (or close enough that that they
            are within a defined tolerance), then the position is accepted as valid for the servo arm.

            This solver iterates through all of the generated possible positions of the servo arm and checks
            their distance to the end point of the upper arm. When this distance is within tolerance of the 
            possible positions of the starting point of the upper arm, this position of the servo arm is
            concluded to be a valid position. If no generated point is within tolerence, the point that is
            closest to a valid position is chosen as the final position.
        '''

        #squared radius of sphere originating
        #on connection of upper arm to platform(to avoid having a sqr root in a for loop)
        squareRadius = link.length**2
        
        #arbitrarily large bounding number for differnece
        minDeltaDist = 10**10 

        for i in range(self.possiblePoints.shape[0]):
            point = self.possiblePoints[i, :]
            squareDist = self.findSquareDist(point, link.endPoint)

            deltaDistance = abs(squareDist - squareRadius)

            if deltaDistance < self.tolerance:
                self.geoEndPoint = point
            elif deltaDistance < minDeltaDist:
                minDeltaDist = deltaDistance
                self.geoEndPoint = point

    def analyticalSolve(self, link):
        self.anaEndPoint = 





def rotationMatrix(rAngs):
    r = R.from_euler('xyz', rAngs, degrees = True)
    rotation = np.array(r.as_matrix())
    return rotation