import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

AVG_WIDTH = 10

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

data_cam = ExpData("data_raw/rev_6_Cam_200_06mm_l1x2.txt", 16, -90)
data_pulley = ExpData("data_raw/Cam_r25mm_n07mm_ldb_2.txt", 20, -96)

print(np.mean(data_cam.motor_current))
print(np.mean(data_pulley.motor_current))

fig, ax = plt.subplots()

motor_current_avg_cam = moving_average(data_cam.motor_current, AVG_WIDTH)
motor_current_avg_pulley = moving_average(data_pulley.motor_current, AVG_WIDTH)

ax.plot(data_cam.time[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)], data_cam.motor_current[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)], 'b+-', label = "cam", linestyle = (0, (1, 10)))
ax.plot(data_pulley.time[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)], data_pulley.motor_current[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)], 'g+-', label = 'round pulley', linestyle = (0, (1, 10)))

ax.plot(data_cam.time[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)+1], motor_current_avg_cam, 'b-', label = "cam moving average", linewidth=3)
ax.plot(data_pulley.time[int(AVG_WIDTH/2):-int(AVG_WIDTH/2)+1], motor_current_avg_pulley, 'g-', label = 'round pulley moving average', linewidth=3)

ax.set_xlabel("Time (ms)")
ax.set_ylabel("Motor current (A)")
#ax.plot(data_pulley.motor_pos, data_pulley.motor_current, '-')
plt.legend()
plt.show()

