import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, root_scalar
from math import sqrt

plt.style.use('dark_background')

# TSA parameters
D = 0.100
R0 = 0.002
A = 0.025
B = 0.006

S = np.array([0.054, 0.217]) # Coordinates of the string separator

# Optimization parameters
XI = 20 # Desired reduction ratio

THETA0 = 3 * 2*np.pi # Motion range lower boundary
THETAM = 8 * 2*np.pi # Motion range upper boundary

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    #return D/(R0*(A+B+R0*theta))
    return sqrt(D**2 + (A+B+R0*theta)**2)/(R0*(A+B+R0*theta))

def tsaLen(theta, theta0):
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

def lenght_criteria_factor(theta0, thetaio, thetai, p, s, df, xi):
    a = moment_normal_point(s, h_fun(thetai), xi) #current moment normal point
    ao = moment_normal_point(s, h_fun(thetaio), xi) #previous moment normal point 
    dlin = tsaLen(thetai, theta0) - tsaLen(thetaio, theta0) #diff in string lenght in the TSA part 

    # f = lambda e: -dlin + np.linalg.norm(e*a + (1-e)*s - p) - e*np.linalg.norm(a - s) + df
    # e_rng = np.arange(0,200, 0.01)
    # f_rng = [f(e) for e in e_rng]

    # print(dlin)

    # plt.plot(e_rng, f_rng)
    # plt.show()

    #find the factor along vector a-s that minimizes the absolute difference between the string in and out the TSA 
    res = root_scalar(lambda e: -dlin + np.linalg.norm(e*a + (1-e)*s - p) + e*np.linalg.norm(a - s) - df, bracket = [0,2])
    print(res.root)

    return res.root

def tangent_criteria_factor(theta, s, p, t, xi):
    """returns how far to move along the (a-s) vector in order to meet the tangent + moment linne criteria"""

    a = moment_normal_point(s, h_fun(theta), xi) #eq_mmt_pt
    v_int = ((p[0] - s[0])*(-t[1]) - (p[1] - s[1])*(-t[0]))/((-t[0])*(s[1] - a[1])- (-t[1])*(s[0] - a[0])) #intersection multiplier along line

    return v_int

def next_cam_pt_and_tgt(e, v, s, a):
    if False:
        #use lenght criteria
        p_new =  s + e*(a-s)
        print("=")
    else:
        #use tangent criteria
        p_new =  s + v*(a-s)

    t_new = (p_new-s)/np.linalg.norm(p_new-s)

    return p_new, t_new

def generate_cam_pts(theta0, theta_max, s, xi, num_pts, offset):
    p0 = moment_normal_point(s, h_fun(theta0), xi)
    t0 = (p0-s)/np.linalg.norm(p0-s)

    p = p0 + offset* t0
    t = t0

    df = np.linalg.norm(p-s)

    cam_pts = [p]

    theta_range = np.linspace(theta0, theta_max, num_pts)
    theta_step = theta_range[1] - theta_range[0]

    gamma_step = 1/xi * theta_step

    for theta in theta_range[1:]:
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        e_int = lenght_criteria_factor(theta0, theta - theta_step, theta, p, s, df, xi)
        v_int = tangent_criteria_factor(theta, s, p, t, xi)

        a = moment_normal_point(s, h_fun(theta), xi)
        p, t = next_cam_pt_and_tgt(e_int, v_int, s, a)
        df = np.linalg.norm(p-s)
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

        ax.plot(0,0,'ro')
        ax.plot(self.s_pos[0], self.s_pos[1], 'go')

        ax.plot([pp[0] for pp in self.pts], [pp[1] for pp in self.pts], '-')
        #ax.grid()
        ax.set_aspect('equal')
        plt.show()

if __name__ == "__main__":
    cam_pts = generate_cam_pts(THETA0, THETAM, S, XI, 50, 0.0)

    fig, ax = plt.subplots()

    ax.plot(0,0,'ro')
    ax.plot(S[0], S[1], 'go')

    ax.plot([pp[0] for pp in cam_pts], [pp[1] for pp in cam_pts], '.-')
    ax.grid()
    ax.set_aspect('equal')
    plt.show()