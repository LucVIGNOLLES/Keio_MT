import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, root_scalar
from math import sqrt

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

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    return D/(R0*(A+B+R0*theta))

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
    return moment_normal_point(s, h_fun(theta0), xi)

def next_cam_pt(theta0, thetaio, thetai, p, s, xi):
    a = moment_normal_point(s, h_fun(thetai), xi)
    ao = moment_normal_point(s, h_fun(thetaio), xi)
    dlin = tsaLen(thetai, theta0) - tsaLen(thetaio, theta0)

    pr = rotate(p, 1/xi*(thetai-thetaio))  

    max_e = fmin(lambda e: -dlin + np.linalg.norm(e*a + (1-e)*s - pr) - e*(np.linalg.norm(a - s) - np.linalg.norm(ao - s)), 0)

    return s + max_e[0]*(a-s)

def generate_cam_pts(theta0, theta_max, s, xi, num_pts, offset):
    p0 = moment_normal_point(s, h_fun(theta0), xi)
    t0 = (p0-s)/np.linalg.norm(p0-s)

    p = p0 + offset* t0

    cam_pts = [p]

    theta_range = np.linspace(theta0, theta_max, num_pts)
    theta_step = theta_range[1] - theta_range[0]

    for theta in theta_range[1:]:
        p = next_cam_pt(theta0, theta-theta_step, theta, p, s, xi)
        cam_pts.append(p)

    return [rotate(p, -i*1/xi*theta_step) for i, p in enumerate(cam_pts)]

if __name__ == "__main__":
    cam_pts = generate_cam_pts(THETA0, THETAM, S, XI, 50, 0.00875)

    fig, ax = plt.subplots()

    ax.plot(0,0,'ro')
    ax.plot(S[0], S[1], 'go')

    ax.plot([pp[0] for pp in cam_pts], [pp[1] for pp in cam_pts], '.-')
    ax.grid()
    ax.set_aspect('equal')
    plt.show()