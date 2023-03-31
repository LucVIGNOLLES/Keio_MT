import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from main import Cam, D, R0, A, B, THETA0, THETAM, THETA_STEP, XI, Y0, S, GAMMA_STEP


def tsaLen(theta, theta0):
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)

def filter_res(gamma_list, theta_list):
    gamma_fil = [gamma_list[0]]
    theta_fil = [theta_list[0]]

    for gamma in gamma_list:
        pass

GAMMA = np.pi/2

#print(cam.bissect_eq_angle(0.14))

def show_cam(cam):
    l, idx = cam.equivalent_string_lenght(GAMMA)
    print(l)

    fig, ax = plt.subplots()

    # plt.plot(a_list, r_list)
    # plt.show()

    ax.plot(0,0,'r.')
    ax.plot(0,SD, 'g.')

    ax.plot([pp[0] for pp in cam.ptsRotated(GAMMA)], [pp[1] for pp in cam.ptsRotated(GAMMA)])
    ax.plot(cam.ptsRotated(GAMMA)[idx][0], cam.ptsRotated(GAMMA)[idx][1], 'o')
    ax.grid()
    ax.set_aspect('equal')
    plt.show()

#show_cam(cam)

# cam = Cam.round(Y0, S, THETA0, THETAM, THETA_STEP, XI)

# l_list = []
# tsa_list = []
# for gamma in np.arange(0, np.pi, 0.001):
#     l, idx = cam.equivalent_string_lenght(gamma)
#     l_list.append(-l+cam.base_string_lenght())
#     tsa_list.append(tsaLen(THETA0 + gamma*1.*XI, THETA0))



# plt.plot(np.arange(0, np.pi, 0.01), l_list, '-')
# plt.plot(np.arange(0, np.pi, 0.01), tsa_list, '-')
# plt.show()



# Main ======

if __name__  == "__main__":

    cam = Cam(Y0, S, THETA0, THETAM, THETA_STEP, XI)

    thetas = np.arange(THETA0, THETAM, THETA_STEP)
    lenghts = [tsaLen(theta, THETA0) for theta in thetas]

    l0, _ = cam.equivalent_string_lenght(0)

    equ_lenghts = [l0 - l for l in lenghts]

    gammas = [cam.bissect_eq_angle(lenght) for lenght in equ_lenghts]

    plt.plot(thetas, gammas, '-')
    plt.plot([0,93], [0, 2.39])
    plt.show()



