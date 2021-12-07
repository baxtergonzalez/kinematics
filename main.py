import numpy as np
import matplotlib.pyplot as plt

#allow animation of plot
plt.figure(1)
plt.ion()
#plt.hold()

def rotate(thetaX):
    """ 
    returns rotation matrix of form:
        |x1 x2|
        |y1 y2| 
    """
    x1 = np.cos(thetaX)
    x2 = -np.sin(thetaX)
    y1 = np.sin(thetaX)
    y2 = np.cos(thetaX)
    
    return x1, x2, y1, y2




#input desired postion and rotation of platform

#position vector for platform
p=np.array([[0.0],[5.0]])
#rotation matrix

#R=np.array([[1.0,0.0],[0.0, 1.0]])
#tX = np.radians(60) #rotation value for platform
#R=np.array([[rotate(tX)[0],rotate(tX)[1]],[rotate(tX)[2], rotate(tX)[3]]])

#design variables

a1=np.array([[-1.0],[0.0]])
a2=np.array([[1.0],[0.0]])

b1=np.array([[-0.5],[0.0]])
b2=np.array([[0.5],[0.0]])

i=0
while i <100:
    #calculate outputs
   
        
    tX = np.radians(3*i/2) #rotation value for platform
    R=np.array([[rotate(tX)[0],rotate(tX)[1]],[rotate(tX)[2], rotate(tX)[3]]])
    
    s1=p+np.dot(R,b1)-a1
    s2=p+np.dot(R,b2)-a2

    Length_s1=np.sqrt(s1[0]**2+s1[1]**2)
    Length_s2=np.sqrt(s2[0]**2+s2[1]**2)
    
    print("Length_s1 = ", Length_s1, " Length_s2 = ", Length_s2, " Rotation = ", tX)
    
    #plot the results
    plt.plot([a1[0],s1[0]+a1[0],p[0],s2[0]+a2[0],a2[0]],[a1[1],s1[1]+a1[1],p[1],s2[1]+a2[1],a2[1]])
    plt.draw()
    plt.pause(0.01)
    plt.cla()
    plt.pause(0.01)
    i=i+1
    if i <25:
        p[0]=p[0]+0.04
    elif i<75:
        p[0]=p[0]-0.04
    else:
        p[0]=p[0]+0.04
plt.ioff()
plt.show()