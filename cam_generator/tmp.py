import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from cam_gen_clean import Cam, D, RS, A, B, THETA0, THETAM, THETA_MAX, inv_eq_lever_arm

def tsaLen(theta, theta0):
    """
    Returns the contration lenght of the TSA in configuration theta relative to configuration theta0
    """
    return sqrt(D**2 + (A+B + RS*theta)**2) - sqrt(D**2 + (A+B + RS*theta0)**2)

fig, ax = plt.subplots()

theta_list = np.linspace(THETA0, THETAM, 100)
h_list = [0.015*inv_eq_lever_arm(theta) for theta in theta_list]

ax.plot(theta_list, h_list, 'g-', linewidth = 3)
ax.set_xlabel("Motor position (Rad)")
ax.set_ylabel("Transmission ratio")

plt.show()






    



