import numpy as np
import matplotlib.pyplot as plt

pre_crop = 2
post_crop = -50

with open(r"C://Users//vigno//Documents//Cours//Keio//MT//experimental_data//cam_200_07mm_ldb.txt") as f:
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
        m_pos.append(int(split[0])*360/1000)
        c_pos.append(int(split[1])*360/4096)
        m_current.append(float(split[2]))
        m_speed.append(float(split[3]))
        load_cell.append(float(split[4]))
        t.append(int(split[5])-t0)

# Find the best firtbetween the motor position and the cam position
polynomial, variance=np.polyfit(m_pos,c_pos,1, full=False, cov = True)
linear_model_fn=np.poly1d(polynomial)

print(variance)
x_s=np.array([0, 30000])

fig, axs = plt.subplots(2)

# axs[0].plot(m_pos, c_pos, '.-', label = "experimental data")
# axs[0].plot([0,80*360], [0, 80*360/200], '--', label = "Target fit")
# axs[0].plot(x_s, linear_model_fn(x_s), '--', label = "Best linear fit")

# axs[0].legend()
axs[0].plot(t, load_cell)

axs[1].plot(t, m_current)

plt.show()
    

