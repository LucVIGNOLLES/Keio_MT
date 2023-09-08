import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData
from tsa import tsaLen

# Fonts for writing on graphs
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

RP = 0.03 #m | counterweight pulley radius 

# Get data from files using the ExpData class
data_cam = ExpData("data_raw/fast_rev_2_Cam_200_06mm_l1x2.txt", 1, -1055)
data_pulley = ExpData("data_raw/fast_rev3_Cam_r30mm_06mm_l1x2.txt", 1, -1100)

#init plot
fig, ax = plt.subplots()

#define arrays for visualizing target line 
x_s= data_cam.motor_pos
y_t= np.array([x/200 for x in x_s])


# find the best fitting linear function for the cam's data 
x, residuals, _, _ = np.linalg.lstsq(np.array(data_cam.cam_pos).reshape(-1,1), data_cam.motor_pos)
print(x)
m = 1/x[0]
print(1/m)
y_s = np.array([x*m for x in x_s])
print(residuals)
print(max(y_s - data_cam.cam_pos))
print(np.mean(y_s - data_cam.cam_pos))

# get the theoretical round pulley motor-cam relation
y_p = [1/RP*tsaLen(x, 0) for x in x_s]

#plot data
ax.plot(data_cam.motor_pos, data_cam.cam_pos, '-', label = "Optimized cam", markersize=8, linewidth = 3)
ax.plot(data_pulley.motor_pos, data_pulley.cam_pos, '-', label = "Circular cam", markersize=8, linewidth = 3)

ax.plot(x_s, y_s, 'g--', label = "Best linear fit")
ax.plot(x_s, y_t, 'r--', label = "Theoretical")

ax.plot(x_s, y_p, '--', label = "Circular theoretical")

ax.set_xlabel("Motor angle (Rad)")
ax.set_ylabel("Cam angle (Rad)")

ax.text(500, 0.65, r'fit : $y = ' + str(int(1/m)) + 'x$', fontdict=font2)
ax.text(340, 0.2, r'Theoretical : $y = 200x$', fontdict=font1)

plt.legend()
plt.show()