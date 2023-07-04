import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

data_cam = ExpData("data_raw/rev_6_Cam_200_06mm_l1x2.txt", 1, -90)
data_pulley = ExpData("data_raw/Cam_r25mm_n07mm_ldb_2.txt", 1, -96)

fig, ax = plt.subplots()

x_s=np.array([0, 700])
y_g=np.array([0, 700/200])

m = np.linalg.lstsq(np.array(data_cam.motor_pos).reshape(-1,1), data_cam.cam_pos)[0][0]
print(1/m)
y_s = np.array([0,700*m])

ax.plot(data_cam.motor_pos, data_cam.cam_pos, '.', label = "Shaped cam")
ax.plot(data_pulley.motor_pos, data_pulley.cam_pos, '.', label = "Round pulley")

ax.plot(x_s, y_s, '--', label = "best linear fit")
ax.plot(x_s, y_g, '--', label = "Target fit")

plt.legend()
plt.show()