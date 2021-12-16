"Simulator of a delta configuration parallel manipulator. Calculates the angle of each actuated joint for an input position based off of design parameters and an input position value using inverse kinematics. This is calculated both geometrically and analytically. The analytical solution works to near 100% accuracy, while the geometric solution works to an accuracy dependant on the resolution requested.
The script also renders out the manipulator using matplotlib. It shows the underlying shapes that yield the geometric solution and give insight into how it is derived. It also validates if the input position is within the work area set by the design parameters of the manipulator.
In the future, input sliders are to be added to allow real time calculation of the angles.

Loosely based on the paper https://www.ohio.edu/mechanical-faculty/williams/html/PDF/DeltaKin.pdf  
