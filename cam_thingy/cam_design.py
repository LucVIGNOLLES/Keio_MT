import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

GAMMA = 0*np.pi
H = 0.00000001
xd, yd = 2, -2

def r(theta, gamma):
    return 1 + 0.2 * np.cos(2*theta + gamma)

def r_cart(theta, gamma):
    return r(theta, gamma)*np.cos(theta), r(theta, gamma)*np.sin(theta)
    
def r_cart_rel(theta, gamma):
    return -r(theta, gamma)*np.cos(theta + gamma), r(theta, gamma)*np.sin(theta + gamma)

def r_der_approx(theta, gamma):
    x, y = r_cart(theta, gamma)
    x_h, y_h = r_cart(theta+H, gamma)

    return (x_h - x)/H, (y_h - y)/H

theta_vec = np.arange(0, 2 * np.pi, .01)[1:]
fig = plt.figure()
ax = fig.add_subplot()

# Make a horizontal slider to control the frequency.
axgamma = plt.axes([0.25, 0.1, 0.65, 0.03])
gamma_slider = Slider(
    ax=axgamma,
    label='Cam angle [Rad]',
    valmin=-np.pi,
    valmax=3*np.pi,
    valinit=np.pi,
)

# The function to be called anytime a slider's value changes
def update(val):
    ax.clear()

    ax.plot(xd, yd, 'o')
    ax.plot([0], [0], 'o')

    x, y = r_cart(theta_vec, gamma_slider.val)
    ax.plot(x, y)

    al_min = 0
    min = 100

    for alpha in np.arange(0, 1 * np.pi, .01)[1:]:
        xa, ya = r_cart(alpha, gamma_slider.val)
        xv, yv = r_der_approx(alpha, gamma_slider.val)

        a = np.array([xa, ya])
        v = np.array([xv, yv])
        d = np.array([xd, yd])

        cross = np.cross(v, d-a)

        if abs(cross) < min:
            min = abs(cross)
            al_min = alpha

    xa_min, ya_min = r_cart(al_min, gamma_slider.val)
    xv_min, yv_min = r_der_approx(al_min, gamma_slider.val)

    alpha_vec = np.arange(al_min+gamma_slider.val, np.pi, .01)[1:]
    x2, y2 = r_cart_rel(alpha_vec, gamma_slider.val)
    ax.plot(x2, y2, 'r-')

    ax.plot([xa_min, xd], [ya_min, yd], 'r-')

    fig.canvas.draw_idle()

# register the update function with each slider
gamma_slider.on_changed(update)

alpha = 1

plt.show()

