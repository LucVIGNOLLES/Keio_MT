import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData

data_cam = ExpData("data_raw/rev_6_Cam_200_06mm_l1x2.txt", 20, -60)
data_pulley = ExpData("data_raw/Cam_r25mm_n07mm_ldb.txt", 1, -100)

fig, ax = plt.subplots()


for data in [data_cam, data_pulley]:
    m_pos = data.motor_pos
    c_pos = data.cam_pos
    D_m_pos = [m_pos[i+1] - m_pos[i] for i in range(len(m_pos)-1)]
    D_c_pos = [c_pos[i+1] - c_pos[i] for i in range(len(c_pos)-1)]

    Xi_approx = []
    t_filtered = []
    m_pos_filtered = []
    for i in range(len(D_m_pos)):
        if D_c_pos[i] != 0:
            Xi_approx.append(D_m_pos[i]/D_c_pos[i])
            t_filtered.append(data.time[i])
            m_pos_filtered.append(c_pos[i])

    ax.plot(t_filtered, Xi_approx, '.-', label = "proposed method")

plt.legend()
plt.show()