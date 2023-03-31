import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import root_scalar
from time import perf_counter
from math import atan2

plt.style.use('dark_background')

# TSA parameters
D = 0.015
R0 = 0.001
A = 0.01
B = 0.0

# Optimization parameters
XI = 15 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 5 * 2*np.pi # Motion range upper boundary
THETA_STEP = 5* 2 * np.pi/200

GAMMA_STEP = 1/XI * THETA_STEP

# Test variables
#P = np.array([0.05, 0]) # First point for the cam
#T = np.array([-2,-1])/np.linalg.norm([-2,-1]) # Tangent on the cam at first point
SD = 0.1 # Distance between the string separator and the cam's COR 
S = np.array([0, SD]) # Coordinates of the string separator

Y0 = 0.00094791

# Work around the unreachable code error
cross = lambda x,y:np.cross(x,y)

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

def equal_moment_x(y, h):
    """
    For a given y, returns the other coordiante x so that the resulting point is both aligned with the cam and the
    strings have the desired moment on it.
    Used to compute the first point of the cam.
    """
    return (XI*(SD - y))/np.sqrt(SD**2*h**2 - XI**2)

def epsilon_solve(e, p, t, h):
    """
    Equation that needs to be solved to find the next point for the cam. 
    The solution epsilon is how much the points need to be moved on the current tangent line 
    of the cam.
    """
    return cross(p + e*t, S)*h/np.linalg.norm(S - (p + e*t)) - XI

def epsilon_solve_fun(p, t, h):
    """
    One parameter version of the epsilon_solve function to be used by the solver
    """
    return lambda e: epsilon_solve(e, p, t, h)

def new_pt_tan_and_angle(p,t,h):
    """
    For given previous point and tangent pair, finds the new point that satisfies the moment criteria,
    according to the current h value (force to motor torque ration of the TSA)
    """
    res = root_scalar(epsilon_solve_fun(p,t,h), bracket=[-20,20])

    e = res.root
    new_p = p + e*t
    new_t = (p-S)/np.linalg.norm(p-S)

    angle = np.pi/2 - np.arccos(np.dot(p, new_p))

    return new_p, new_t, angle

def generate_cam_pts(y0, S, theta0, thetaM, theta_step,  xi):
    p0 = np.array([equal_moment_x(y0, h_fun(theta0)), y0])
    t0 = (p0-S)/np.linalg.norm(p0-S)

    p = p0
    t = t0

    p_list = [p]
    t_list = [t]

    gamma_step = 1/xi * theta_step

    for theta in np.arange(theta0, thetaM, theta_step):
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        p, t, a = new_pt_tan_and_angle(p, t, h_fun(theta))

        p_list.append(p)
        t_list.append(t)

    return [rotate(p, -i*gamma_step) for i, p in enumerate(p_list)]

def generate_round_pts(y0, S, theta0, thetaM, theta_step,  xi):
    p0 = np.array([equal_moment_x(y0, h_fun(theta0)), y0])
    t0 = (p0-S)/np.linalg.norm(p0-S)

    p = p0
    t = t0

    p_list = [p]
    t_list = [t]

    gamma_step = 1/xi * theta_step

    for theta in np.arange(theta0, thetaM, theta_step):
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        p, t, a = new_pt_tan_and_angle(p, t, h_fun(theta))

        p_list.append(p)
        t_list.append(t)

    base_pts = [rotate(p, -i*gamma_step) for i, p in enumerate(p_list)]
    norms = [np.linalg.norm(pt) for pt in base_pts]
    mean = np.mean(norms)

    return [rotate(np.array([mean, 0]), (-i+10)*gamma_step) for i, p in enumerate(p_list)]

## ======

class Cam:
    def __init__(self, y0, S, theta0, thetaM, theta_step,  xi) -> None:
        self.pts = generate_cam_pts(y0, S, theta0, thetaM, theta_step, xi)
        self.s_pos = S
        self.alpha_step = 1/xi * theta_step

        self.gamma0 = atan2(self.pts[0][1], self.pts[0][0])
        self.gammaf = atan2(self.pts[-1][1], self.pts[-1][0])

        self.gammaM = 2*np.pi - self.gammaf - self.gamma0

        self.num_pts = len(self.pts)

    @classmethod
    def round(cls, y0, S, theta0, thetaM, theta_step,  xi):
        cls.pts = generate_round_pts(y0, S, theta0, thetaM, theta_step, xi)
        cls.s_pos = S
        cls.alpha_step = 1/xi * theta_step

        cls.gamma0 = atan2(cls.pts[0][1], cls.pts[0][0])
        cls.gammaf = atan2(cls.pts[-1][1], cls.pts[-1][0])

        cls.gammaM = 2*np.pi - cls.gammaf - cls.gamma0

        cls.num_pts = len(cls.pts)

        return cls


    def ptsRotated(self, gamma):
        """
        Returns a version of the outline points rotated by an angle gamma in the direct direction
        """
        return [rotate(pt, gamma) for pt in self.pts]

    def perim(self, idx1, idx2):
        """
        Simple perim approximation. Returns the sum of all lenght of segments between two specified indexes
        """
        assert idx1 <= idx2, "idx1 should be greater than idx2"

        perim = 0

        for i in range(idx1, idx2-1):
            perim += np.linalg.norm(self.pts[i+1] - self.pts[i])

        return perim

    def base_string_lenght(self):
        return np.linalg.norm(self.pts[0] - self.s_pos) + self.perim(0, self.num_pts-1)

    def equivalent_string_lenght(self, gamma):
        """
        Returns the lenght of string that would be on the cam side if the cam has been rotated by angle gamma
        """

        # Find contact point by checking which outline point
        # creates the max angle from the separator point
        angles = [atan2Vec(p-self.s_pos) for p in self.ptsRotated(gamma)]
        idx =  angles.index(max(angles))

        return np.linalg.norm(self.ptsRotated(gamma)[idx] - self.s_pos) + self.perim(idx, self.num_pts-1), idx

    def equivalent_angle(self, lenght, step = 0.1):
        gamma = 0
        while True:
            l, _ = self.equivalent_string_lenght(gamma)
            if l < lenght:
                break

            gamma += step

        return gamma

    def bissect_eq_angle(self, lenght, eps = 1e-4):
        gamma_min = 0
        gamma_max = np.pi

        gamma = self.gammaM/2

        while True:
            l, _ = self.equivalent_string_lenght(gamma)
            if l < lenght:
                gamma_max = gamma
                gamma = gamma_min + (gamma-gamma_min)/2
            else:
                gamma_min = gamma
                gamma = gamma + (gamma_max - gamma)/2

            if abs(l - lenght) < eps:
                break

        return gamma

# Main

if __name__ == "__main__":
    p0 = np.array([equal_moment_x(Y0, h_fun(THETA0)), Y0])
    t0 = (p0-S)/np.linalg.norm(p0-S)

    p = p0
    t = t0

    p_list = [p]
    t_list = [t]

    for theta in np.arange(THETA0, THETAM, THETA_STEP):
        p = rotate(p, GAMMA_STEP)
        t = rotate(t, GAMMA_STEP)

        p, t, a = new_pt_tan_and_angle(p, t, h_fun(theta))

        p_list.append(p)
        t_list.append(t)

    pp_list = [rotate(p, -i*GAMMA_STEP) for i, p in enumerate(p_list)]

    cam = Cam(Y0, S, THETA0, THETAM, THETA_STEP, XI)

    pp_list = cam.pts

    # r_list = [np.linalg.norm(pp) for pp in pp_list]
    # a_list = [atan2(pp[1], pp[0]) for pp in pp_list]

    fig, ax = plt.subplots()

    # plt.plot(a_list, r_list)
    # plt.show()

    ax.plot(0,0,'ro')
    ax.plot(0,SD, 'go')

    ax.plot([pp[0] for pp in pp_list], [pp[1] for pp in pp_list], '.-')
    ax.grid()
    ax.set_aspect('equal')
    plt.show()

    with open("cam_outline.txt", 'w') as f:
        for p in pp_list:
            f.write(str(1000*p[0]) + ' ' + str(1000*p[1]) + ' ' + str(0) + '\n') # 1000 foactor converts to mm


