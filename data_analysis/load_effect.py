import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

paths_cams = ["data_raw/Cam_200_n07mm_l0.txt",
            "data_raw/Cam_200_n07mm_l251.txt",
            "data_raw/Cam_200_n07mm_l505.txt",
            "data_raw/Cam_200_n07mm_l947.txt",
            "data_raw/Cam_200_n07mm_l1361.txt",]

paths_pulleys = ["data_raw/Cam_r30mm_n07mm_l0.txt",
                "data_raw/Cam_r30mm_n07mm_l251.txt",
                "data_raw/Cam_r30mm_n07mm_l505.txt",
                "data_raw/Cam_r30mm_n07mm_l947.txt",
                "data_raw/Cam_r30mm_n07mm_l1361.txt",]

fig, ax = plt.subplots()

for i, path in enumerate(paths_cams):
    data = ExpData(path, 1, -100)
    ax.plot(data.motor_pos, data.cam_pos, '+--', label = "Shaped cam " + str(i))
    
for i, path in enumerate(paths_pulleys):
    data = ExpData(path, 1, -100)
    ax.plot(data.motor_pos, data.cam_pos, '+--', label = "Shaped  " + str(i))


plt.legend()
plt.show()