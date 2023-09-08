import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, root_scalar
from math import sqrt

### First, parameters for the TSA and cam. Should be changed if the system is changed  

# TSA geometric parameters
D = 0.300 #m
RS = 0.36e-3 #m
A = 0.010 #m
B = 0.002 #m

# Coordinates of the string separator relative to the cam's COR, in meter
S = np.array([0.0605, 0.3625 - D]) 

# Optimization parameters
XI = 200 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 110 * 2*np.pi # Motion range upper boundary

# Theoretical max angle for the motor before overtwisting
THETA_MAX = (D*np.pi- 2*(A+B))/(2*RS)

### Utility functions

def rotate(u, angle):
    """
    For a 2D vector, rotates it by a given angle in the direct direction
    """
    return np.array([u[0]*np.cos(angle) - u[1]*np.sin(angle), u[0]*np.sin(angle) + u[1]*np.cos(angle)])


### TSA model fuctions

def inv_eq_lever_arm(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta 
    This is the inverse of the equivalent lever arm described in the report
    """
    return sqrt(D**2 + (A+B+RS*theta)**2)/(RS*(A+B+RS*theta))

def tsaLen(theta, theta0 = THETA0):
    """
    Returns the contration lenght of the TSA in configuration theta relative to configuration theta0
    """
    return sqrt(D**2 + (A+B + RS*theta)**2) - sqrt(D**2 + (A+B + RS*theta0)**2)


### Cam points placement functions

def normal_lever_arm_point(s, h, xi):
    """
    Returns the point where the force from the strings has the desired moment on the cam, and is normal to the cam's equivalent radius
    """
    r_eq = xi/h #r0*h = Cam torque / motor torque = xi

    # find the absolute angle to the point using root solver
    res = root_scalar(lambda b0: s[0]*np.cos(b0) + s[1]*np.sin(b0) - r_eq, bracket=[-np.pi/4,np.pi/4]) 
    abs_angle = res.root 

    # Return the coresponding coordinates for the point with desired lever arm normal to the equivalent circle
    return np.array([r_eq*np.cos(abs_angle), r_eq*np.sin(abs_angle)])

def first_cam_pt(theta0, s, xi):
    """Returns the first cam point, simply the moment normal point at motor angle theta 0.
    Might need to be changed to something else if the algorithm requires a different statring point
    
    Edit: it doesn't make much of a difference to change it"""
    return normal_lever_arm_point(s, inv_eq_lever_arm(theta0), xi)

def tangent_criteria_factor(theta, s, p, t, xi):
    """returns how far to move along the (e-s) vector in order to find the intersection point of the pulling line and the tangent to the previous point"""

    e = normal_lever_arm_point(s, inv_eq_lever_arm(theta), xi) #eq_mmt_pt
    v_int = ((p[0] - s[0])*(-t[1]) - (p[1] - s[1])*(-t[0]))/((-t[0])*(s[1] - e[1])- (-t[1])*(s[0] - e[0])) #intersection multiplier along line

    return v_int

def generate_cam_pts(theta0, theta_max, s, xi, num_pts, offset):
    """
    Sequencially generate a cam's outline.
    """

    #Initial point and tangent
    p0 = normal_lever_arm_point(s, inv_eq_lever_arm(theta0), xi)
    t0 = (p0-s)/np.linalg.norm(p0-s)

    #Optional offset in the direction of the tangent
    p = p0 + offset* t0
    t = t0

    #List container for the cam's points
    cam_pts = [p]

    #List of all motor configurations to be evaluated. One cam point per configuration
    theta_range = np.linspace(theta0, theta_max, num_pts)
    theta_step = theta_range[1] - theta_range[0]

    gamma_step = 1/xi * theta_step #amount to rotate the cam between each step

    #For each motor angle:
    for theta in theta_range[1:]:
        #rotate previous point and tangent
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        #find intersection between the rotated tangent and the equal moment line
        v_int = tangent_criteria_factor(theta, s, p, t, xi)

        #find the next point and tangent
        e = normal_lever_arm_point(s, inv_eq_lever_arm(theta), xi)
        p = s + v_int*(e-s)
        t = (p-s)/np.linalg.norm(p-s)

        # Add new point to the list
        cam_pts.append(p)

    # Rotate all points back to their original position
    return [rotate(p, -i*1/xi*theta_step) for i, p in enumerate(cam_pts)]

# A class to make defining a cam more convinient
# Previous functions are used to generate the cams attributes
class Cam:
    def __init__(self, s, theta0, thetaM, num_pts, xi, offset):
        self.pts = generate_cam_pts(theta0, thetaM, s, xi, num_pts, offset)
        self.s_pos = s
        self.alpha_step = 1/xi *((thetaM - theta0) / num_pts)

        self.xi = xi

        self.num_pts = num_pts

    def ptsRotated(self, angle):
        """
        Returns a version of the outline points rotated by an angle gamma in the direct direction
        """
        return [rotate(pt, angle) for pt in self.pts]

    def perim(self, idx1, idx2):
        """
        Returns the sum of all lenght of segments between two specified indexes
        """
        assert idx1 <= idx2, "idx1 should be greater than idx2"

        perim = 0

        for i in range(idx1, idx2-1):
            perim = perim + np.linalg.norm(self.pts[i+1] - self.pts[i])

        return perim

    def outStringLen(self, pt_idx):
        """
        When the contact point of the strings on the cam is pt_idx, returns the lenght of string rolled around the cam
        plus the lenght of string between the contact point and the separator
        """
        assert pt_idx < self.num_pts, "pt_idx out of range"

        perim = self.perim(pt_idx, self.num_pts - 1)
        pts = self.ptsRotated(pt_idx*self.alpha_step)
        line = np.linalg.norm(pts[pt_idx] - self.s_pos)

        return perim + line
    
    def show(self):
        """
        Simple function to display the cam in a plot
        """
        fig, ax = plt.subplots()

        ax.plot(0,0,'ro', label = "Cam COR")
        ax.plot(self.s_pos[0], self.s_pos[1], 'go')

        ax.plot([pp[0] for pp in self.pts], [pp[1] for pp in self.pts], '.-', label = "Cam outline", linewidth=2)
        ax.grid()
        ax.set_aspect('equal')
        ax.set_xlabel("x coordinate (m)")
        ax.set_ylabel("y coordinate (m)")
        plt.legend()
        plt.show()

if __name__ == "__main__":
    # Define a cam
    cam = Cam(S, THETA0, THETAM, 100, XI, 0)
    cam.show()

    # Store the cam points in a .csv file, with custom name
    filename = "cams/cam_" + str(XI) + "_" + str(D).strip("0.") + "_" + str(RS).strip("0.") + "_" + str(A).strip("0.") + "_" + str(B).strip("0.") + ".csv"
    with open(filename, 'w') as f:
        for p in cam.pts:
            f.write(str(1000*p[0]) + ',' + str(1000*p[1]) + ',' + str(0) + '\n') # 1000 foactor converts to mm