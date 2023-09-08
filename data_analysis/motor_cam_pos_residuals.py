import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData
from tsa import tsaLen

font1 = {'family': 'serif',
        'color':  'darkred',
        'weight': 'normal',
        'size': 16,
        }

font2 = {'family': 'serif',
        'color':  'darkgreen',
        'weight': 'normal',
        'size': 16,
        }


RP = 0.025 #m | pulley radius

# Get data from files
data_cam = ExpData("data_raw/fast_rev_13_Cam_200_06mm_l1x2.txt", 1, -101)
data_pulley = ExpData("data_raw/fast_rev5_Cam_r30mm_06mm_l1x2.txt", 1, -150)

#init plot
fig, ax = plt.subplots()

#define arrays for visualizing target line 
y_traget_cam=np.array([x/200 for x in data_cam.motor_pos])
y_traget_pulley=np.array([1/RP*tsaLen(x, 0) for x in data_pulley.motor_pos])


# find the best fitting linear function for the cam's data 
res = np.linalg.lstsq(np.array(data_cam.motor_pos).reshape(-1,1), data_cam.cam_pos)
m = res[0][0]
print(1/m)

y_residuals_cam = [cam_pos - model_pos for cam_pos, model_pos in zip(data_cam.cam_pos, y_traget_cam)]
y_residuals_pulley = [pulley_pos - model_pos for pulley_pos, model_pos in zip(data_pulley.cam_pos, y_traget_pulley)]

#plot data
ax.plot(data_cam.motor_pos, y_residuals_cam, '-', label = "Optimized cam", markersize=8, linewidth = 3)
ax.plot(data_pulley.motor_pos, y_residuals_pulley, '-', markersize=8, label = "Round pulley", linewidth = 3)

ax.set_xlabel("Motor angle (Rad)")
ax.set_ylabel("Residual error to the respective models (Rad)")

plt.legend()
plt.show()