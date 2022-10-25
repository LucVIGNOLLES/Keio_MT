import numpy as np
import matplotlib.pyplot as plt
from random import randint
from cam import Cam
from tsa import Tsa
from actuator import Actuator 

# TODO: Get rid of the iNdIviDuAl class, and turn all of its components into a single "evaluate(actuator)" function. ====== OK
# Add an option to generate an actuator with random values for the variables
# Code the evolutive methods in the Population class

class Population:
    def __init__(self, population_sz) -> None:
        self.pop_size = population_sz
        self.individuals = []
            

    def mate():
        return 0

    def evolve():
        return 0

# Testing ======

GAMMA0 = 0
GAMMAM = np.pi

if __name__ == "__main__":
    cam = Cam([0.8, 1.1, 1.3, 3, 2, 1.7, 1.6], 1.8, 0.3)
    tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
    actu = Actuator(cam, tsa, 0.15, -0.2)


    val = actu.evaluate(GAMMA0, GAMMAM, 0.02, np.pi/92)

    print(val)

    plt.figure(1)
    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        x, y = cam.r_cart(theta, 0)
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.plot(0,0, '.')

    # plt.figure(2)
    # plt.plot(gamma_lst, theta_lst)
    # # plt.plot(gamma_lst, alpha_lst)
    # # plt.plot(gamma_lst, l_lst)
    # plt.plot([GAMMA0, GAMMAM], [indiv.actu.tsa.theta_max, indiv.actu.tsa.theta_max], 'r-')
    plt.show() 

    