import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from cam_gen import Cam, D, R0, A, B
from cam_gen_tangent import CamTgtCriteria

# S = np.array([0.05, 0.1])


# theta0 = 0*2*np.pi
# thetam = 13*2*np.pi

# xi = 25
# cam1 = Cam(np.array([0.05, 0.1]), theta0, thetam, 100, xi, 0)

# cam1.show()
# s0 = cam1.outStringLen(0)

# idx = []
# s_lst = []

# for i in range(cam1.num_pts):
#     s = cam1.outStringLen(i)
#     idx.append(i)
#     s_lst.append(s - s0)

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    #return D/(R0*(A+B+R0*theta))
    return (D**2 + R0**2*theta**2)/(R0**2*theta)

theta_lst = np.arange(2*np.pi, 50*2*np.pi, 0.1)
h1_lst = [D/(R0*(A+B+R0*theta)) for theta in theta_lst]
h2_lst = [sqrt(D**2 + (A+B+R0*theta)**2)/(R0*(A+B+R0*theta)) for theta in theta_lst]

plt.plot(theta_lst, h1_lst, label = "h1")
plt.plot(theta_lst, h2_lst, label = "h2")
plt.legend()
plt.show()






    



