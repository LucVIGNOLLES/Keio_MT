import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from cam_gen import Cam, D, A, B, R0

theta0 = 0*2*np.pi

def tsaLen(theta, theta0):
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)

for i in range(10, 50):
    try:
        cam = Cam(np.array([0.05, 0.1]), theta0, 10*2*np.pi, 500, i, 0)
        print("xi = " + str(i) + " => " + str(cam.perim(0, len(cam.pts))))
    except Exception as e:
        print(e)





    



