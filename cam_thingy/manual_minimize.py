import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.gridspec as gridspec

from cam import Cam
from tsa import Tsa
from actuator import Actuator

# Global parameters
GAMMA0 = 0 # Start angle of the cam
GAMMAM = np.pi # End angle of the cam
STEP_SZ = 0.1 # Gamma angle step

GOAL = 20 # Goal reduction ratio

# Tsa parameters
D = 0.2
R0 = 0.003
A = 0.01
B = 0

TSA = Tsa(D, R0, A, B)

XS = 0.15
YS = -0.2

CAM = Cam([0.01088736, 0.00902949, 0.00940026, 0.01240981, 0.01975054, 0.03287334,
                0.04894144, 0.06176267, 0.06186754, 0.04512916, 0.03005022, 0.01838828], 
                2, 
                0.21948481997605881)

def update(val):
    cam_x, cam_y = [], []
    for alpha in np.arange(0, 2*np.pi, 0.1):
        x, y = actuator.cam.r_cart(alpha, 0)
        cam_x.append(x)
        cam_y.append(y)
    
    cam_line


if __name__ == "__main__":
    cam = Cam([0.01088736, 0.00902949, 0.00940026, 0.01240981, 0.01975054, 0.03287334,
                0.04894144, 0.06176267, 0.06186754, 0.04512916, 0.03005022, 0.01838828], 
                2, 
                0.21948481997605881)

    actuator = Actuator(cam, TSA, XS, YS)

    cam_x, cam_y = [], []
    for alpha in np.arange(0, 2*np.pi, 0.1):
        x, y = actuator.cam.r_cart(alpha, 0)
        cam_x.append(x)
        cam_y.append(y)

    e, gm, lh = actuator.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL, True)

    fig, axs = plt.subplots(2)

    fig.subplots_adjust(bottom = 0.35)

    axs[0].set_aspect('equal')

    cam_line, = axs[0].plot(cam_x, cam_y)
    center, = axs[0].plot(0, 0, '.r')

    ratio, = axs[1].plot(gm, lh)
    base, = axs[1].plot(gm, np.ones_like(gm)*20)

    def generic_updater(kpt_idx):
        return lambda val: update(val, kpt_idx)

    def update(val, kpt_idx):
        new_cam = cam
        print(cam.keypoints)
        new_cam.keypoints[kpt_idx] = val*new_cam.keypoints[kpt_idx]

        new_actu = Actuator(new_cam, TSA, XS, YS)

        cam_x, cam_y = [], []
        for alpha in np.arange(0, 2*np.pi, 0.1):
            x, y = new_actu.cam.r_cart(alpha, 0)
            cam_x.append(x)
            cam_y.append(y)

        cam_line.set_xdata(cam_x)
        cam_line.set_ydata(cam_y)

        e, gm, lh = new_actu.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL, True)
        ratio.set_xdata(gm)
        ratio.set_ydata(lh)


    # Make a horizontal sliders
    kpt_axes = [fig.add_axes([0.25, 0.05 + 0.02*i, 0.65, 0.01]) for i in range(len(actuator.cam.keypoints))]
    kpt_sliders = [Slider(
            ax=ax,
            label=f'Keypoint {i}',
            valmin=0.9,
            valmax=1.1,
            valinit=1,
        ) for i, ax in enumerate(kpt_axes)]
    
    for i, slider in enumerate(kpt_sliders):
        slider.on_changed(generic_updater(i))
        
    plt.show()