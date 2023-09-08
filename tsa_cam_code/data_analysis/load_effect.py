import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

def calculate_max_disparity(lists):
    max_disparity = []
    for i in range(len(lists[0])):
        point_disparity = max(abs(lists[0][i] - lists[1][i]), abs(lists[0][i] - lists[2][i]), abs(lists[0][i] - lists[3][i]))
        max_disparity.append(point_disparity)
    return max_disparity


paths_cams = ["data_raw/Cam_200_n07mm_l251.txt",
            "data_raw/Cam_200_n07mm_l505.txt",
            "data_raw/Cam_200_n07mm_l947.txt",
            "data_raw/Cam_200_n07mm_l1361.txt",]


paths_pulleys = ["data_raw/Cam_r30mm_n07mm_l251.txt",
                "data_raw/Cam_r30mm_n07mm_l505.txt",
                "data_raw/Cam_r30mm_n07mm_l947.txt",
                "data_raw/Cam_r30mm_n07mm_l1361.txt",]

# crops for each data file.
post_crops_cams = [-123, -105, -123, -107]
post_crops_pulleys = [-181, -115, -103, -127]

# load for each experiement in mN
loads = ["213", "338", "555", "758"]


fig, ax = plt.subplots()

cams = []
pulleys = []

# plot all the data
for i, path in enumerate(paths_cams):
    data = ExpData(path, 1, post_crops_cams[i])
    ax.plot(data.motor_pos, data.cam_pos, '-', label = "Cam | " + loads[i] + "mNm")
    print("cam" + str(i))
    print("")

    cams.append(data)
    
for i, path in enumerate(paths_pulleys):
    data = ExpData(path, 1, post_crops_pulleys[i])
    ax.plot(data.motor_pos, data.cam_pos, '--', label = "Pulley | " + loads[i] + "mNm")
    print("pulley" + str(i))
    print("")

    pulleys.append(data)

ax.set_xlabel("Motor angle (Rad)")
ax.set_ylabel("Cam / Pulley angle (Rad)")

plt.legend()
plt.show()

plt.plot