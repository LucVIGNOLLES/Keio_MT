import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import root_scalar
from time import perf_counter

# TSA parameters
D = 0.2
R0 = 0.0015
A = 0.1
B = 0.0

# optimization parameters
XI = 30

THETA0 = 15 * 2*np.pi
THETAM = 30 * 2*np.pi
THETA_STEP = 0.01

GAMMA_STEP = 1/XI * THETA_STEP

# Test variables
P = np.array([0.05, 0])
T = np.array([-2,-1])/np.linalg.norm([-2,-1])
SD = 0.2
S = np.array([0, SD])

Y0 = 0.02

# Work around the unreachable code error
cross = lambda x,y:np.cross(x,y)

def h_fun(theta):
    return D/(R0*(A+B+R0*theta))

def rotate(u, angle):
    return np.array([u[0]*np.cos(angle) - u[1]*np.sin(angle), u[0]*np.sin(angle) + u[1]*np.cos(angle)])

def equal_moment_x(y, h):
    return (XI*(SD - y))/np.sqrt(SD**2*h**2 - XI**2)

def epsilon_solve(e, p, t, h):
    return cross(p + e*t, S)*h/np.linalg.norm(S- p - e*t) - XI

def epsilon_solve_fun(p, t, h):
    return lambda e: epsilon_solve(e, p, t, h)

def new_pt_tan_and_angle(p,t,h):
    res = root_scalar(epsilon_solve_fun(p,t,h), bracket=[-20,20])

    e = res.root
    new_p = p + e*t
    new_t = (p-S)/np.linalg.norm(p-S)

    angle = np.pi/2 - np.arccos(np.dot(p, new_p))

    return new_p, new_t, angle

# Main

if __name__ == "__main__":
    p0 = np.array([equal_moment_x(Y0, h_fun(THETA0)), Y0])
    t0 = (p0-S)/np.linalg.norm(p0-S)

    p = p0
    t = t0

    p_list = [p]
    t_list = [t]
    a_list = [0]

    for theta in np.arange(THETA0, THETAM, THETA_STEP):
        p = rotate(p, GAMMA_STEP)
        t = rotate(t, GAMMA_STEP)

        p, t, a = new_pt_tan_and_angle(p, t, h_fun(theta))

        p_list.append(p)
        t_list.append(t)
        a_list.append(a)

    pp_list = []

    for i, p in enumerate(p_list):
        pp_list.append(rotate(p, -i*GAMMA_STEP))

    fig, ax = plt.subplots()

    ax.plot(0,0,'r.')
    ax.plot(0,SD, 'g.')

    ax.plot([pp[0] for pp in pp_list], [pp[1] for pp in pp_list])
    ax.grid()
    ax.set_aspect('equal')
    plt.show()


