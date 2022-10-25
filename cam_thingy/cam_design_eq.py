import numpy as np
import matplotlib.pyplot as plt
from cam import Cam

def norm2(u,v):
    return np.sqrt((v[0] - u[0])**2 + (v[1] - u[1])**2)

GAMMA = 0*np.pi # Current angle of the cam
H = 0.00000001 # Step for approximate derivative computation
xd, yd = 2, -20 # x,y coordinates of the fixed point for the rope to come out of

cam = Cam([0.7, 1, 1.5, 3, 1.7, 1.2, 0.5], 2)

plt_list = []
gamma_range = np.arange(0 , 2 * np.pi, .5)[1:]

for gamma in gamma_range:
    alpha_min = 0
    min = 100

    for alpha in np.arange(-np.pi/4 - gamma, np.pi/2 - gamma, .1)[1:]:
        xa, ya = cam.r_cart(alpha, gamma)
        xv, yv = cam.r_der_approx(alpha, gamma)

        a = np.array([xa, ya])
        v = np.array([xv, yv])
        d = np.array([xd, yd])

        cross = np.cross(v, d-a)

        if abs(cross) < min:
            min = abs(cross)
            alpha_min = alpha

    xa_min, ya_min = cam.r_cart(alpha_min, gamma)

    len1 = norm2((d[0], d[1]), (xa_min, ya_min))
    len2 = cam.r_int_approx(alpha_min, np.pi, gamma)

    plt_list.append(len1 + len2)

plt.plot(gamma_range, plt_list)
# plt.plot([0, 0], [0, 5], 'r-')
# plt.plot([np.pi*2, np.pi*2], [0, 5], 'r-')
plt.show()