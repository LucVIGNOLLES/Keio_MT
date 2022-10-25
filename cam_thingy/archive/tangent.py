from xml.sax.handler import all_properties
import numpy as np
import matplotlib.pyplot as plt

def r(alpha):
    return 2+np.cos(alpha)

def r_dot(alpha):
    return -np.sin(alpha)

def a_vec(alpha):
    return r(alpha)*np.cos(alpha), r(alpha)*np.sin(alpha)

def a_dot(alpha):
    return r_dot(alpha)*np.cos(alpha) - r(alpha)*np.sin(alpha), r_dot(alpha)*np.sin(alpha) + r(alpha)*np.cos(alpha)


# Plot utilities
alpha_vec = np.arange(0, 2*np.pi, 0.05)
xa_vec, ya_vec = a_vec(alpha_vec)

xad, yad = a_dot(np.pi/4)
xa, ya = a_vec(np.pi/4)

plt.plot(xa_vec, ya_vec)
plt.plot(0,0, '.g')
plt.plot([xa, xa + xad], [ya, ya + yad], '-r')
plt.show()




