import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import root_scalar
from time import perf_counter
from math import atan2, sqrt

plt.style.use('dark_background')

# TSA parameters
D = 0.05
R0 = 0.001
A = 0.01
B = 0.0

# Optimization parameters
XI = 20 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 10 * 2*np.pi # Motion range upper boundary
THETA_STEP = 5* 2 * np.pi/200

GAMMA_STEP = 1/XI * THETA_STEP

S = np.array([0.05, 0.1]) # Coordinates of the string separator

#General functions

def atan2Vec(u):
    return atan2(u[1], u[0])

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    return D/(R0*(A+B+R0*theta))

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

def new_pt_tan_and_angle(s, p, t, h, xi):
    """
    For given previous point and tangent pair, finds the new point that satisfies the moment criteria,
    according to the current h value (force to motor torque ration of the TSA)
    """
    a = moment_normal_point(s, h, xi) #eq_mmt_pt
    u_int = ((s[0] - p[0])*(s[1] - a[1]) - (s[1] - p[1])*(s[0] - a[0]))/((s[0] - a[0])*(-t[1]) - (s[1] - a[1])*(-t[0])) #intersection multiplier along line

    p_new = p+u_int*t
    t_new = (p_new-s)/np.linalg.norm(p_new-s)

    angle = np.pi/2 - np.arccos(np.dot(p, p_new))

    return p_new, t_new, angle

def generate_cam_pts(s, theta0, thetaM, num_pts,  xi, offset):
    p0 = moment_normal_point(s, h_fun(theta0), xi)
    t0 = (p0-s)/np.linalg.norm(p0-s)

    p = p0 + offset* t0 #first cam point, can be at an offset from the normal to the COR
    t = t0

    # init lists of points and tangents
    p_list = [p]
    t_list = [t]

    # Theoretical relation between theta and gamma
    theta_step = (thetaM - theta0) / num_pts
    gamma_step = 1/xi * theta_step

    for theta in np.arange(theta0, thetaM, theta_step):
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        p, t, a = new_pt_tan_and_angle(s, p, t, h_fun(theta), xi)

        p_list.append(p)
        t_list.append(t)

    return [rotate(p, -i*gamma_step) for i, p in enumerate(p_list)]

class Cam:
    def __init__(self, s, theta0, thetaM, num_pts, xi, offset):
        self.pts = generate_cam_pts(s, theta0, thetaM, num_pts, xi, offset)
        self.s_pos = s
        self.alpha_step = 1/xi *((thetaM - theta0) / num_pts)

        self.xi = xi

        self.gamma0 = atan2(self.pts[0][1], self.pts[0][0])
        self.gammaf = atan2(self.pts[-1][1], self.pts[-1][0])

        self.gammaM = 2*np.pi - self.gammaf - self.gamma0

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

if __name__ == "__main__":
    fig, ax = plt.subplots()
    for xi in range(26, 27):
        try:
            cam = Cam(S,THETA0, xi/12*THETAM, 500, xi, 0)

            test_idx = 0

            print(cam.outStringLen(test_idx))

            ax.plot(0,0,'ro')
            ax.plot(S[0], S[1], 'go')

            ax.plot([pp[0] for pp in cam.ptsRotated(test_idx*cam.alpha_step)], [pp[1] for pp in cam.ptsRotated(test_idx*cam.alpha_step)], '.-')
            #ax.plot([pp[0] for pp in cam2.ptsRotated(test_idx*cam2.alpha_step)], [pp[1] for pp in cam2.ptsRotated(test_idx*cam2.alpha_step)], '.-')
        except Exception as e:
            print(e)
    ax.grid()
    ax.set_aspect('equal')
    plt.show()