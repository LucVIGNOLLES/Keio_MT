import numpy as np
import matplotlib.pyplot as plt

from cam import Cam
from tsa import Tsa
from actuator import Actuator

# Global parameters
GAMMA0 = 0 # Start angle of the cam
GAMMAM = np.pi # End angle of the cam
STEP_SZ = 0.1 # Gamma angle step

GOAL = 20 # Goal reduction ratio

# Tsa parameters
D = 0.2
R0 = 0.003
A = 0.01
B = 0

TSA = Tsa(D, R0, A, B)

XS = 0.15
YS = -0.2

def snail_minimize(actu, num_steps, amp = 0.001):
    test_cam = actu.cam
    test_actu = actu

    current_val = actu.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL)
    print(f"Initiating with value {current_val}")
    print("\n")

    for i in range(num_steps):
        for j, kpt in enumerate(actu.cam.keypoints):
            print(f"Step {i}, keypoint {j} ...")
            test_cam.keypoints[j] = kpt + amp*kpt
            test_actu = Actuator(test_cam, TSA , XS, YS)

            new_val = test_actu.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL)

            if new_val <= current_val:
                actu = test_actu
                current_val = new_val

            test_cam.keypoints[j] = kpt - amp*kpt
            test_actu = Actuator(test_cam, TSA , XS, YS)

            new_val = test_actu.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL)

            if new_val <= current_val:
                actu = test_actu
                current_val = new_val

            test_cam = actu.cam
            
            print(f"Finished with value {current_val}")
    return actu


if __name__ == "__main__":
    cam = Cam([0.01088736, 0.00902949, 0.00940026, 0.01240981, 0.01975054, 0.03287334,
                0.04894144, 0.06176267, 0.06186754, 0.04512916, 0.03005022, 0.01838828], 
                2, 
                0.21948481997605881)

    actuator = Actuator(cam, TSA, XS, YS)

    res = snail_minimize(actuator, 2, 1)

    print(res.cam.keypoints)
