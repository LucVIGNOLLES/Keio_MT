import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, root_scalar
from math import sqrt

#plt.style.use('dark_background')

# TSA parameters
D = 0.200
R0 = 0.36e-3
A = 0.020
B = 0.002

S = np.array([0.0605, 0.3625 - D]) # Coordinates of the string separator

# Optimization parameters
XI = 200 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 110 * 2*np.pi # Motion range upper boundary

THETA_MAX = (D*np.pi- 2*(A+B))/(2*R0)

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    return sqrt(D**2 + (A+B+R0*theta)**2)/(R0*(A+B+R0*theta))

def tsaLen(theta, theta0):
    """
    Returns the contration lenght of the TSA in configuration theta relative to configuration theta0
    """
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)

def rotate(u, angle):
    """
    For a 2D vector, rotates it by a given angle in the direct direction
    """
    return np.array([u[0]*np.cos(angle) - u[1]*np.sin(angle), u[0]*np.sin(angle) + u[1]*np.cos(angle)])

# Cam points placement functions

def moment_normal_point(s, h, xi):
    """
    Returns the point where the force from the strings has the desired moment on the cam, and is normal to the cam's radius
    """
    r_eq = xi/h #r0*h = Cam torque / motor torque = xi

    res = root_scalar(lambda b0: s[0]*np.cos(b0) + s[1]*np.sin(b0) - r_eq, bracket=[-np.pi/4,np.pi/4]) # find the absolute angle to the point
    beta = res.root

    return np.array([r_eq*np.cos(beta), r_eq*np.sin(beta)])

def first_cam_pt(theta0, s, xi):
    """Returns the first cam point, simply the moment normal point at motor angle theta 0.
    Might need to be changed to something else if the algorithm requires a different statring point"""
    return moment_normal_point(s, h_fun(theta0), xi)

def tangent_criteria_factor(theta, s, p, t, xi):
    """returns how far to move along the (a-s) vector in order to meet the tangent + moment linne criteria"""

    a = moment_normal_point(s, h_fun(theta), xi) #eq_mmt_pt
    v_int = ((p[0] - s[0])*(-t[1]) - (p[1] - s[1])*(-t[0]))/((-t[0])*(s[1] - a[1])- (-t[1])*(s[0] - a[0])) #intersection multiplier along line

    return v_int

def generate_cam_pts(theta0, theta_max, s, xi, num_pts, offset):
    """
    Sequencially generate a cam's outline.
    """

    #Initial point and tangent
    p0 = moment_normal_point(s, h_fun(theta0), xi)
    t0 = (p0-s)/np.linalg.norm(p0-s)

    #Optional offset in the direction of the tangent
    p = p0 + offset* t0
    t = t0

    #Container for the cam's points
    cam_pts = [p]

    #List of all motor configurations to be evaluated. One cam point per configuration
    theta_range = np.linspace(theta0, theta_max, num_pts)
    theta_step = theta_range[1] - theta_range[0]

    gamma_step = 1/xi * theta_step #amount to rotate the cam between each step

    for theta in theta_range[1:]:
        #rotate previous point and tangent
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        #find intersection between the rotated tangent and the equal moment line
        v_int = tangent_criteria_factor(theta, s, p, t, xi)

        #find the next point and tangent
        a = moment_normal_point(s, h_fun(theta), xi)
        p = s + v_int*(a-s)
        t = (p-s)/np.linalg.norm(p-s)

        cam_pts.append(p)

    return [rotate(p, -i*1/xi*theta_step) for i, p in enumerate(cam_pts)]

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
        Simple perim approximation. Returns the sum of all lenght of segments between two specified indexes
        """
        assert idx1 <= idx2, "idx1 should be greater than idx2"

        perim = 0

        for i in range(idx1, idx2-1):
            perim = perim + np.linalg.norm(self.pts[i+1] - self.pts[i])

        return perim

    def outStringLen(self, pt_idx):
        assert pt_idx < self.num_pts, "pt_idx out of range"

        perim = self.perim(pt_idx, self.num_pts - 1)
        pts = self.ptsRotated(pt_idx*self.alpha_step)
        line = np.linalg.norm(pts[pt_idx] - self.s_pos)

        return perim + line
    
    def show(self):
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
    cam = Cam(S, THETA0, THETAM, 100, XI, 0)
    cam.show()

    filename = "cams/cam_" + str(XI) + "_" + str(D).strip("0.") + "_" + str(R0).strip("0.") + "_" + str(A).strip("0.") + "_" + str(B).strip("0.") + ".csv"

    with open(filename, 'w') as f:
        for p in cam.pts:
            f.write(str(1000*p[0]) + ',' + str(1000*p[1]) + ',' + str(0) + '\n') # 1000 foactor converts to mm