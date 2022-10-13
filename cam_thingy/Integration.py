import numpy as np
from Tsa import Tsa
from cam_gen import Cam
import matplotlib.pyplot as plt

XS = 0.15
YS = -0.2

GAMMA0 = 0
GAMMAM = np.pi

def norm2(u,v):
    return np.sqrt((v[0] - u[0])**2 + (v[1] - u[1])**2)

def l_cam(cam, xs, ys, gamma):
    alpha_min = 0
    min = 100

    for alpha in np.arange(-np.pi/4 - gamma, np.pi/2 - gamma, .1)[1:]:
        xa, ya = cam.r_cart(alpha, gamma)
        xv, yv = cam.r_der_approx(alpha, gamma)

        a = np.array([xa, ya])
        v = np.array([xv, yv])
        s = np.array([xs, ys])

        cross = np.cross(v, s-a)

        if abs(cross) < min:
            min = abs(cross)
            alpha_min = alpha

    xa_min, ya_min = cam.r_cart(alpha_min, gamma)

    len1 = norm2((s[0], s[1]), (xa_min, ya_min))
    len2 = cam.r_int_approx(alpha_min, np.pi, gamma)

    return len1 + len2

def l_rel_cam(cam, xs, ys, gamma):
    return l_cam(cam, xs, ys, gamma) - l_cam(cam, xs, ys, 0)

# Main ======

if __name__ == "__main__":

    cam = Cam([2.5, 2, 1.5, 1, 1.5, 1.7, 2], 2, 0.5)
    actu = Tsa(0.3, 0.003, 0.01, 0, 1)

    gamma_range = np.arange(GAMMAM, GAMMA0, -0.05)
    theta_list = []

    for gamma in gamma_range:
        l = l_rel_cam(cam, XS, YS, gamma)
        theta_list.append(actu.theta(l))

    plt.plot([GAMMA0, GAMMAM], [actu.theta_max, actu.theta_max], 'r-')
    plt.plot(gamma_range, theta_list)
    plt.show()

    