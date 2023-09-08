import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

### Import Cam classe and fixed parameters for covinience.
from cam_gen_clean import Cam, D, A, B, RS

theta0 = 0*2*np.pi
thetam = 90*2*np.pi

def tsaLen(theta, theta0):
    """
    Returns the contracted length of the TSA for a given motor angle compared to any initial angle
    """
    return sqrt(D**2 + (A+B + 0.97*RS*theta)**2) - sqrt(D**2 + (A+B + 0.97*RS*theta0)**2)

# Create a cam object 
xi = 200
cam1 = Cam(np.array([0.054, 0.217]), theta0, thetam, 20, xi, 0)
cam1.show()
l0 = cam1.outStringLen(0)

# Compute a bunch of lengths to compare, then graph them 
theta_step = cam1.xi*cam1.alpha_step
theta_list = [theta0+i*theta_step for i in range(cam1.num_pts)]

len_in_lst = [tsaLen(theta, theta0) for theta in theta_list]
len_out_lst_cam1 = [cam1.outStringLen(i) - l0 for i in range(cam1.num_pts)]

len_total_lst = [lin + lout for lin, lout in zip(len_in_lst, len_out_lst_cam1)]

fig, (ax1, ax2) = plt.subplots(1, 2)

ax1.plot(theta_list[:-1], len_out_lst_cam1[:-1], label = 'lout', linewidth = 2)
ax1.plot(theta_list[:-1], len_in_lst[:-1], label = 'lin', linewidth = 2)
ax1.plot(theta_list[:-1], len_total_lst[:-1], label = 'tot', linewidth = 2)
ax1.set_xlabel("Motor angle (Rad)")
ax1.set_ylabel("Lenght (m)")
ax1.legend()

ax2.plot(theta_list[:-1], len_total_lst[:-1],'g', label = 'tot', linewidth = 2)
ax2.set_xlabel("Motor angle (Rad)")
ax2.set_ylabel("Lenght (m)")

plt.show()
