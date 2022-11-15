import numpy as np
import matplotlib.pyplot as plt
from random import randint
from cam import Cam
from tsa import Tsa
from actuator import Actuator, plot_actu
from multiprocessing import Pool
from functools import partial

# This approach did not work last time i tried, but it might still be worth a try with the force linearization

def adapt_perim_dir(actu, gamma0, gammam, step_sz, goal_coeff, h, base_value):
    """
    Finds if th eperimeter needs tobe increased or decreased in order to improve the score 
    Not very evolved though, proportions could be improved
    """
    new_perim = actu.cam.perim + actu.cam.perim*h

    actu.cam = Cam(actu.cam.keypoints, 2, new_perim)

    new_val = actu.evaluate_force(gamma0, gammam, step_sz, goal_coeff)

    return base_value - new_val

def get_mod_value(actu, gamma0, gammam, step_sz, goal_coeff, h, base_value, i):
    """
    Finds the influence of increasing a keypoints distance ont the score. 
    Gives a positive value if yes, and a negative one if no
    """
    new_kpts =  actu.cam.keypoints
    new_kpts[i] = actu.cam.keypoints[i] + actu.cam.keypoints[i]*h
    new_actu = Actuator(Cam(new_kpts, 2, actu.cam.perim), actu.tsa, actu.xs, actu.ys)
    return base_value - new_actu.evaluate_force(gamma0, gammam, step_sz, goal_coeff)

def find_downhill_vector(actu, gamma0, gammam, step_sz, goal_coeff, h, base_value):
    """
    Gets the influence of modifying each keypoint and stores it into an array to be used as a direction for modifying the cam shape
    """
    values = []
    with Pool(len(actu.cam.keypoints)) as p:
        values = p.map(partial(get_mod_value, actu, gamma0, gammam, step_sz, goal_coeff, h, base_value), range(len(actu.cam.keypoints)))

    values = np.array(values)
    direction = values/np.linalg.norm(values)

    return direction

def minimize(actu, gamma0, gammam, step_sz, goal_coeff, h, evol_ratio1, evol_ratio2, num_steps):
    """
    Sequencially modifies the perimeter and the keypoints a number of times in order to try to decrease the score value
    """
    for k in range(num_steps):
        # print("====> Performing step",  k+1)
        # val = actu.evaluate_force(gamma0, gammam, step_sz, goal_coeff)
        # print(">> All changes val :", val)

        # perim_dir = adapt_perim_dir(actu, gamma0, gammam, step_sz, goal_coeff, h, val)
        # new_cam = Cam(actu.cam.keypoints, 2, actu.cam.perim + actu.cam.perim*perim_dir*evol_ratio1)

        # actu.cam = new_cam

        val = actu.evaluate_force(gamma0, gammam, step_sz, goal_coeff)
        dir = find_downhill_vector(actu, gamma0, gammam, step_sz, goal_coeff, h, val)

        new_keypoints = []
        
        for i, ui in enumerate(dir):
            new_keypoints.append(actu.cam.keypoints[i] + ui*evol_ratio2*actu.cam.keypoints[i])

        new_cam = Cam(new_keypoints, 2, actu.cam.perim)

        actu.cam = new_cam

        plot_actu(actu, k+1, val)
        print(">> Perim change val : ", val, "; Perimeter : ", actu.cam.perim)
        print("  ")
    return actu    

## Main ======

if __name__ == "__main__":
    tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
    cam = Cam([2, 3, 7, 9, 10, 10, 5, 2], 2, 0.3)
    actu = Actuator(cam, tsa, 0.15, -0.2)

    # dir = find_downhill_vector(actu, 0, np.pi, 0.1, np.pi/92, 0.001)
    # print(dir)

    actu = minimize(actu, 0, np.pi, 0.1, np.pi/92, 1e-6, 0.5, 0.01, 20)
    print(actu.cam.keypoints)