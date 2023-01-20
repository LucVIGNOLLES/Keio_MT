import numpy as np
import matplotlib.pyplot as plt

from main import Cam, Y0, S, THETA0, THETAM, THETA_STEP, XI, SD

GAMMA = np.pi/5

cam = Cam(Y0, S, THETA0, THETAM, THETA_STEP, XI)

print(cam.equivalent_angle(0.14, 0.01))

def show_cam(cam):
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

l_list = []
for gamma in np.arange(0, np.pi, 0.01):
    l, idx = cam.equivalent_string_lenght(gamma)
    l_list.append(l)

plt.plot(np.arange(0, np.pi, 0.01), l_list, '-')
plt.show()
