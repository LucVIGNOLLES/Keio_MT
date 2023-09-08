import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

paths_cams = ["data_raw/Cam_200_n07mm_l251.txt",
            "data_raw/Cam_200_n07mm_l505.txt",
            "data_raw/Cam_200_n07mm_l947.txt",
            "data_raw/Cam_200_n07mm_l1361.txt",]


paths_pulleys = ["data_raw/Cam_r30mm_n07mm_l251.txt",
                "data_raw/Cam_r30mm_n07mm_l505.txt",
                "data_raw/Cam_r30mm_n07mm_l947.txt",
                "data_raw/Cam_r30mm_n07mm_l1361.txt",]

path = "data_raw/Cam_r30mm_n07mm_l1361.txt"


fig, ax = plt.subplots()

data = ExpData(path, 1, -127)
ax.plot(data.motor_pos, data.cam_pos, '+--', label = "Shaped cam")

plt.legend()
plt.show()