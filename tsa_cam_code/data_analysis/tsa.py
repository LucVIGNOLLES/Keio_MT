import numpy as np
from math import sqrt

# TSA parameters
D = 0.200
R0 = 0.27e-3
A = 0.020
B = 0.002

S = np.array([0.0605, 0.3625 - D]) # Coordinates of the string separator

# Optimization parameters
XI = 200 # Desired reduction ratio

THETA0 = 0 * 2*np.pi # Motion range lower boundary
THETAM = 110 * 2*np.pi # Motion range upper boundary

THETA_MAX = (D*np.pi- 2*(A+B))/(2*R0)

def h_fun(theta):
    """
    Returns the string force to motor torque ration for a given twist state theta
    """
    return sqrt(D**2 + (A+B+R0*theta)**2)/(R0*(A+B+R0*theta))

def tsaLen(theta, theta0):
    """
    Returns the contration lenght of the TSA in configuration theta relative to configuration theta0
    """
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)
