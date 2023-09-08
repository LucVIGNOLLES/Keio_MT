import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

AVG_WIDTH = 7 # Should be odd
ALIGN_INDEX = int((AVG_WIDTH-1)/2)
 
def moving_average(x, w):
    """
    Returns a new list where each data point has been averaged by its w nearest neighbors
    new list is smaller of size len(x) - w
    """
    return np.convolve(x, np.ones(w), 'valid') / w

data_cam = ExpData("data_raw/fast_rev_11_Cam_200_06mm_l1x2.txt", 416, -800)
data_pulley = ExpData("data_raw/fast_rev1_Cam_r30mm_06mm_l1x2.txt", 420, -800)

print(np.mean(data_cam.motor_current))
print(np.mean(data_pulley.motor_current))

fig, ax = plt.subplots()

diff = [cc - cp for cc,cp in zip(data_cam.motor_current, data_pulley.motor_current)]
#ax.plot(diff, "+")
ax.plot(moving_average(diff,7))
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Current difference (A)")

print(np.mean(diff))

plt.legend()
plt.grid()
plt.show()
