import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import root_scalar
from math import sqrt

# TSA parameters
D = 0.200
R0 = 0.0015
A = 0.035
B = 0.006

S = np.array([0.054, 0.217]) # Coordinates of the string separator

# Optimization parameters
XI = 50 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 28 * 2*np.pi # Motion range upper boundary

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

if __name__ == "__main__":  

    fig = plt.figure()
    ax = plt.axes(xlim =(-0.07, 0.1), 
                ylim =(-0.07, 0.07))
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid()
    ims = []

    num_pts = 20
    offset = 0

    #Initial point and tangent
    p0 = moment_normal_point(S, h_fun(THETA0), XI)
    t0 = (p0-S)/np.linalg.norm(p0-S)

    line_end = S + 1.5*(p0-S)
    tg_end = p0 + 0.05*t0

    im_line,  = ax.plot([S[0], line_end[0]], [S[1], line_end[1]], '-r', label = "String pulling line")
    im_circle = ax.add_patch(plt.Circle((0, 0), XI/h_fun(THETA0), color='g', fill = False))
    im_normal, = ax.plot([0, p0[0]], [0, p0[1]], 'g-', label = "Equivalent lever arm")
    im_cam, = plt.plot(p0[0], p0[1], '-b', label = "Cam outline")
    im_point, = plt.plot(p0[0], p0[1], 'Db')
    im_center, = ax.plot(0,0, '.r', label = "Cam COR")
    im_tg, = plt.plot([p0[0], tg_end[0]], [p0[1], tg_end[1]], '--b')

    ims.append([im_center, im_line, im_circle, im_normal, im_cam, im_point, im_tg])

    #Optional offset in the direction of the tangent
    p = p0 + offset* t0
    t = t0

    #Container for the cam's points
    cam_pts = [p]

    #List of all motor configurations to be evaluated. One cam point per configuration
    theta_range = np.linspace(THETA0, THETAM, num_pts)
    theta_step = theta_range[1] - theta_range[0]

    gamma_step = 1/XI * theta_step #amount to rotate the cam between each step

    for k, theta in enumerate(theta_range[1:]):
        #rotate previous point and tangent
        p = rotate(p, gamma_step)
        t = rotate(t, gamma_step)

        tg_end = p + 0.05*t
        im_tg, = plt.plot([p[0], tg_end[0]], [p[1], tg_end[1]], '--b')

        cam_pts = [rotate(p, 1/XI*theta_step) for p in cam_pts]

        #find intersection between the rotated tangent and the equal moment line
        v_int = tangent_criteria_factor(theta, S, p, t, XI)

        #find the next point and tangent
        a = moment_normal_point(S, h_fun(theta), XI)
        p = S + v_int*(a-S)
        t = (p-S)/np.linalg.norm(p-S)

        #add plots to animation
        line_end = S + 1.5*(a-S)

        im_line,  = ax.plot([S[0], line_end[0]], [S[1], line_end[1]], '-r')
        im_circle = ax.add_patch(plt.Circle((0, 0), XI/h_fun(theta), color='g', fill = False))
        im_normal, = ax.plot([0, a[0]], [0, a[1]], 'g-')
        im_cam, = ax.plot([pp[0] for pp in cam_pts], [pp[1] for pp in cam_pts], 'bD-', linewidth = 2)
        im_center, = ax.plot(0,0, '.r')

        ims.append([im_center, im_line, im_circle, im_normal, im_cam, im_tg])

        cam_pts.append(p)
        tg_end = p + 0.05*t
        im_tg, = plt.plot([p[0], tg_end[0]], [p[1], tg_end[1]], '--b')
        im_cam, = ax.plot([pp[0] for pp in cam_pts], [pp[1] for pp in cam_pts], 'bD-', linewidth = 2)


        ims.append([im_center, im_line, im_circle, im_normal, im_cam, im_tg])

    ani = animation.ArtistAnimation(fig, ims, interval = 300)
    #plt.legend()
    plt.show()
    ani.save("results/cam_gen.mp4")