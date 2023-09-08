import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, atan2

from cam_gen_clean import Cam, S, THETA0, THETAM, XI, inv_eq_lever_arm, THETA_MAX, rotate

cam = Cam(S, THETA0, THETAM, 100, XI, 0)

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

pts = [rotate(pp, np.pi-1.23) for pp in cam.pts]

r_list = [1000*np.linalg.norm(pp) for pp in pts]
angle_list = [atan2(pp[0],pp[1]) for pp in pts]

ax.plot(angle_list, r_list, '-', linewidth=3)
ax.grid()
ax.set_aspect('equal')
ax.text(0.4, 1.3*np.max(r_list), 'Radius (mm)', va='bottom', ha='center')
plt.grid()
plt.show()