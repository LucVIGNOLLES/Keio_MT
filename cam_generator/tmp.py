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


# paths = [r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_r30mm_n07mm_l0.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_r30mm_n07mm_l251.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_r30mm_n07mm_l505.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_r30mm_n07mm_l947.txt",
#          r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_r30mm_n07mm_l1361.txt"]

paths = [r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_n07mm_l1361.txt"]

fig, axs = plt.subplots(2)
#axs = [ax]

for path in paths:
    m_pos, c_pos, mc, ms, lc, t = extract_data(path, 1, -10)

    # Find the best fit between the motor position and the cam position
    polynomial, variance=np.polyfit(m_pos, c_pos,1, full=False, cov = True)
    linear_model_fn=np.poly1d(polynomial)

    x_s=np.array([0, 600])
    axs[0].plot(m_pos, c_pos, '.-', label = "experimental data cam")


theta_list = np.linspace(THETA0, THETAM, 100)
len_list = [1/0.05*tsaLen(theta, THETA0) for theta in theta_list]

axs[0].plot([0,200*np.pi], [0, 200*np.pi/200], '--', label = "Target fit")
axs[0].plot(x_s, linear_model_fn(x_s), '--', label = "Best linear fit")
axs[0].plot(theta_list, len_list, '--', label = "Previous method")
axs[0].legend()

axs[1].plot(t, mc)

plt.show()






    



