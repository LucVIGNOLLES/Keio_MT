import numpy as np
from actuator import Actuator
from evolution import mate
from cam import Cam
from tsa import Tsa

cam = Cam([0.8, 1.1, 1.3, 3, 2, 1.7, 1.6], 2, 0.5)
tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
actu = Actuator(cam, tsa, 0.15, -0.2)

cam2 = Cam([0.3, 2.1, 2.3, 3.5, 1.3, 1.7, 1.6], 2, 0.8)
tsa2 = Tsa(0.2, 0.003, 0.01, 0, 1)
actu2 = Actuator(cam2, tsa2, 0.15, -0.2)

child = mate(actu, actu2)

print(child.cam.keypoints)
print(child.cam.perim)



