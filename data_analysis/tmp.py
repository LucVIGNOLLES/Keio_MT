import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from cam_gen_clean import Cam, D, R0, A, B, THETA0, THETAM, THETA_MAX

def tsaLen(theta, theta0):
    """
    Returns the contration lenght of the TSA in configuration theta relative to configuration theta0
    """
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)

def extract_data(path, pre_crop, post_crop):
    with open(path) as f:
        lines = f.readlines() #Get lines from csv / txt file

        lines = lines[pre_crop:post_crop] # crop unwanted data.

        # Containers for formating and storing the raw data into
        m_pos = []
        c_pos = []

        m_current = []
        m_speed = []

        load_cell = []

        t = []
        t0 = int(lines[0].split(',')[5])

        # Extract and format the data
        for line in lines:
            split = line.split(',')
            m_pos.append(int(split[0])*np.pi*2/1000)
            c_pos.append(int(split[1])*np.pi*2/4096)
            m_current.append(float(split[2]))
            m_speed.append(float(split[3]))
            load_cell.append(float(split[4]))
            t.append(int(split[5])-t0)

    return m_pos, c_pos, m_current, m_speed, load_cell, t 


# paths = [r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_l0.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_l251.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_l505.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_l947.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_l1361.txt"]

paths = [r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_ldb.txt",
         r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//round_exp.txt"]

fig, axs = plt.subplots(2)
#axs = [ax]

for path in paths:
    m_pos, c_pos, mc, ms, lc, t = extract_data(path, 1, -25)

    # Find the best fit between the motor position and the cam position
    polynomial, variance=np.polyfit(m_pos, c_pos,1, full=False, cov = True)
    linear_model_fn=np.poly1d(polynomial)

    x_s=np.array([0, 600])
    #axs[0].plot(m_pos, c_pos, '.-', label = "experimental data cam")

    D_m_pos = [m_pos[i+1] - m_pos[i] for i in range(len(m_pos)-1)]
    D_c_pos = [c_pos[i+1] - c_pos[i] for i in range(len(c_pos)-1)]

    Xi_approx = []
    t_filtered = []
    m_pos_filtered = []
    for i in range(len(D_m_pos)):
        if D_c_pos[i] != 0:
            Xi_approx.append(D_m_pos[i]/D_c_pos[i])
            t_filtered.append(t[i])
            m_pos_filtered.append(c_pos[i])

    axs[0].plot(t_filtered, Xi_approx, '.-', label = "proposed method")
    axs[1].plot(t_filtered, m_pos_filtered)

# theta_list = np.linspace(THETA0, THETAM, 100)
# len_list = [25*tsaLen(theta, THETA0) for theta in theta_list]

# axs[0].plot([0,200*np.pi], [0, 200*np.pi/200], '--', label = "Target fit")
# axs[0].plot(x_s, linear_model_fn(x_s), '--', label = "Best linear fit")
# axs[0].plot(theta_list, len_list, '--', label = "Previous method")
# axs[0].legend()

D_m_pos = [m_pos[i+1] - m_pos[i] for i in range(len(m_pos)-1)]
D_c_pos = [c_pos[i+1] - c_pos[i] for i in range(len(c_pos)-1)]

Xi_approx = []
t_filtered = []
for i in range(len(D_m_pos)):
    if D_c_pos[i] != 0:
        Xi_approx.append(-D_m_pos[i]/D_c_pos[i])
        t_filtered.append(t[i])

ref = 200*np.ones_like(t_filtered)

axs[0].plot(t_filtered, Xi_approx, label = "previous method")
axs[0].plot(t_filtered, ref, label = "target")
axs[0].legend()
plt.show()






    



