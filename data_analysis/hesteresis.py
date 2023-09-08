import numpy as np
import matplotlib.pyplot as plt
from experiment_data import ExpData
from tsa import tsaLen

font1 = {'family': 'serif',
        'color':  'darkgreen',
        'weight': 'normal',
        'size': 16,
        }

font2 = {'family': 'serif',
        'color':  'orange',
        'weight': 'normal',
        'size': 16,
        }


RP = 0.025 #m | pulley radius

# Get data from files
data_cam = ExpData("data_raw/rev_4_Cam_200_06mm_l1x2.txt", 1, -21)
#data_cam = ExpData("data_raw/Cam_r30mm_n07mm_ldb.txt", 1, -21)

#init plot
fig, ax = plt.subplots()

#define arrays for visualizing target line 
x_s=np.array(range(0, 700))
y_t=np.array([x/200 for x in x_s])


# find the best fitting linear function for the cam's data 
m = np.linalg.lstsq(np.array(data_cam.motor_pos).reshape(-1,1), data_cam.cam_pos)[0][0]
print(1/m)
y_s = np.array([x*m for x in x_s])

# get the theoretical round pulley motor-cam relation
y_p = [1/RP*tsaLen(x, 0) for x in x_s]

#plot data
ax.plot(data_cam.motor_pos, data_cam.cam_pos, '+-', label = "Optimized cam", markersize=8, linewidth = 3)

ax.plot(x_s, y_s, '--', label = "Cam best linear fit")
ax.plot(x_s, y_t, '--', label = "Cam Theoretical")

ax.set_xlabel("Motor angle (Rad)")
ax.set_ylabel("Cam  angle (Rad)")

ax.text(500, 0.65, r'fit : $x = ' + str(int(1/m)) + 'y$', fontdict=font2)
ax.text(340, 0.2, r'Theoretical : $x = 200y$', fontdict=font1)

plt.legend()
plt.show()