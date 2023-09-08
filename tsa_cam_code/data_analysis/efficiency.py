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

# data_cam = ExpData("data_raw/fast_rev_11_Cam_200_06mm_l1x2.txt", 395, -800)
# data_pulley = ExpData("data_raw/fast_rev3_Cam_r30mm_06mm_l1x2.txt", 120, -800)

data_cam = ExpData("data_raw/fast_rev_11_Cam_200_06mm_l1x2.txt", 130, -800)
data_pulley = ExpData("data_raw/fast_rev1_Cam_r30mm_06mm_l1x2.txt", 120, -850)

print(np.mean(data_cam.motor_current))
print(np.mean(data_pulley.motor_current))

fig, ax = plt.subplots(2)

motor_current_avg_cam = moving_average(data_cam.motor_current, AVG_WIDTH)
motor_current_avg_pulley = moving_average(data_pulley.motor_current, AVG_WIDTH)

#ax[0].plot(data_cam.time[ALIGN_INDEX:-ALIGN_INDEX], np.array(data_cam.motor_current[ALIGN_INDEX:-ALIGN_INDEX]), '+-', label = "Optimized cam", linestyle = (0, (1, 10)), color = "tab:blue")
#ax[0].plot(data_pulley.time[ALIGN_INDEX:-ALIGN_INDEX], np.array(data_pulley.motor_current[ALIGN_INDEX:-ALIGN_INDEX]), '+-', label = 'Circular cam', linestyle = (0, (1, 10)), color = "tab:orange")

ax[0].plot(data_cam.time[ALIGN_INDEX:-ALIGN_INDEX], motor_current_avg_cam, '-', linewidth=3, color = "tab:blue", label = "Optimized cam")
ax[0].plot(data_pulley.time[ALIGN_INDEX:-ALIGN_INDEX], motor_current_avg_pulley, 'g-', linewidth=3, color = "tab:orange", label = 'Circular cam')

ax[0].set_xlabel("Time (ms)")
ax[0].set_ylabel("Motor current (A)")

ax[0].legend()

# data_cam = ExpData("data_raw/fast_rev_11_Cam_200_06mm_l1x2.txt", 406, -800)
# data_pulley = ExpData("data_raw/fast_rev1_Cam_r30mm_06mm_l1x2.txt", 410, -800)

print(np.mean(data_cam.motor_current))
print(np.mean(data_pulley.motor_current))

diff = []
t_filtered = []
for i in range(min(len(motor_current_avg_cam), len(motor_current_avg_pulley))):
    diff.append(motor_current_avg_cam[i] - motor_current_avg_pulley[i])
    t_filtered.append(data_cam.time[i])
#ax.plot(diff, "+")
ax[1].plot(data_cam.time[ALIGN_INDEX:-ALIGN_INDEX], diff, color = "tab:purple")
ax[1].set_xlabel("Time (ms)")
ax[1].set_ylabel("Current difference (A)")

ax[1].grid()

plt.show()

