from ssl import ALERT_DESCRIPTION_HANDSHAKE_FAILURE
import numpy as np
import matplotlib.pyplot as plt
from cam_gen import Cam

def euler_min(cam, gamma, xf, yf):
    """
    Only works for very smooth cams, otherwise  there are too many local minimas
    """
    alpha = np.pi/3
    al_min = 0
    al_max = np.pi/2
    cross = 1
    while abs(cross) > 0.01:
        xa, ya = cam.r_cart(alpha, gamma)
        xv, yv = cam.r_der_approx(alpha, gamma)

        a = np.array([xa, ya])
        v = np.array([xv, yv])
        f = np.array([xf, yf])

        cross = np.cross(v, f-a)
        print(cross)

        if cross < 0:
            al_max = alpha
            alpha = (alpha - al_min/2)
        else:
            al_min = alpha
            alpha = (al_max - alpha)/2

    return alpha

# Testing =====

if __name__ == "__main__":
    cam = Cam([1, 1, 1.5, 3, 1.7, 1.2, 1], 1.8)
    gamma = 0
    xf, yf = 3, -2

print(euler_min(cam, gamma, xf, yf))