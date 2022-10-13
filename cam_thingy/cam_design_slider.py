import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from cam_gen import Cam

xd, yd = 0.15, -0.2 # Coordinates of the fixed point

cam = Cam([1, 1.2, 1.3, 1.2, 1.1, 1, 1], 1, 0.05)

theta_vec = np.arange(0, 2 * np.pi, .05)[1:]
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
    ax.set_xlim([-0.3,0.3])
    ax.set_ylim([-0.4, 0.2])

    ax.plot(xd, yd, 'go')
    ax.plot([0], [0], 'ro')
    ax.plot([0, 0.3*np.cos(gamma_slider.val)], [0, 0.3*np.sin(gamma_slider.val)], 'r-')
    x = []
    y = []
    for theta in theta_vec:
        xth, yth = cam.r_cart(theta, gamma_slider.val)
        x.append(xth)
        y.append(yth)
    ax.plot(x, y)

    al_min = 0
    min = 100

    # Search for the point where the string contacts the cam
    # This version looks for a minimum of cross product between the tangent to the cam at a given 
    # point and the straight line between that point and the fixed point 
    for alpha in np.arange(-np.pi/4 - gamma_slider.val, np.pi/2 - gamma_slider.val, .1)[1:]:
        xa, ya = cam.r_cart(alpha, gamma_slider.val)
        xv, yv = cam.r_der_approx(alpha, gamma_slider.val)

        a = np.array([xa, ya])
        v = np.array([xv, yv])
        d = np.array([xd, yd])

        cross = np.cross(v, d-a)

        if abs(cross) < min:
            min = abs(cross)
            al_min = alpha

    # TODO switch to a more optimized algorithm
    # - Euler's method
    # - "Ray tracing" from the fixed point until there's only one intersection point with the cam

    xa_min, ya_min = cam.r_cart(al_min, gamma_slider.val)
    xv_min, yv_min = cam.r_der_approx(al_min, gamma_slider.val)

    alpha_vec = np.arange(al_min, np.pi, .05)[1:]
    x2 = []
    y2 = []

    # plot the lenght of string that's on the cam plus the one between the cam and the fixed point.
    for alpha in alpha_vec:
        x2al, y2al = cam.r_cart(alpha, gamma_slider.val)
        x2.append(x2al)
        y2.append(y2al)

    ax.plot(x2, y2, 'r-', linewidth = 3)
    ax.plot([xa_min, xd], [ya_min, yd], 'r-', linewidth = 3)

    fig.canvas.draw_idle()

# register the update function with each slider
gamma_slider.on_changed(update)
plt.show()

