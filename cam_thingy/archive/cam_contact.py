import numpy as np
import matplotlib.pyplot as plt
from cam import Cam
from matplotlib.widgets import Slider

plt.style.use('dark_background')

DEBUG = 0

def bisection(cam, gamma, xf, yf, alpha_min = -2*np.pi, alpha_max = 4*np.pi, cross_thresh = 1e-5, max_step = 1000):
    """
    Only works for very smooth cams, otherwise  there are too many local minimas
    Edit : now it actually works... kinda
    Edit2 : It breaks sometimes because we need to account for periodicity
    Edit3 : periodicity is delt with, but sometimes it breks for ne apparent reason
    Edit4 : okay, now it works perfectly, even though the results seem a bit irregular
    """

    # Base values for bisection
    alpha = 3
    al_min = alpha_min
    al_max = alpha_max
    cross = np.array([0,0,1])
    steps = 0

    while abs(cross[2]) > cross_thresh and steps < max_step:
        steps = steps + 1

        # Compute point and tangent
        xa, ya = cam.r_cart(alpha, gamma)
        xv, yv = cam.r_der_approx(alpha, gamma)

        # Compute vectors
        a = np.array([xa, ya, 0])
        v = np.array([xv, yv, 0])
        f = np.array([xf, yf,0])

        # Cross product to discriminate
        # Doing it the other way make the program find the other 0 
        cross = np.cross(v, f-a)

        # Modify alpha and boundaries according to the result.
        if cross[2] > 0:
            al_max = alpha 
            alpha = alpha - (alpha - al_min)/2
        else:
            al_min = alpha
            alpha = alpha + (al_max - alpha)/2 

    return alpha % (2*np.pi)

def naive_min(cam, gamma, xf, yf, alpha_min = -np.pi/4, alpha_max = np.pi/2):
    """
    Very inneficient and imprecise version of the above algorithm
    """
    alpha_res = 0
    min = 100

    for alpha in np.arange(alpha_min - gamma, alpha_max - gamma, .1)[1:]:
        xa, ya = cam.r_cart(alpha, gamma)
        xv, yv = cam.r_der_approx(alpha, gamma)

        a = np.array([xa, ya, 0])
        v = np.array([xv, yv, 0])
        s = np.array([xf, yf, 0])

        cross = np.cross(v, s-a)

        if abs(cross[2]) < min:
            min = abs(cross[2])
            alpha_res = alpha

    return alpha_res


# Testing =====

if __name__ == "__main__":

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

    cam = Cam([1, 1, 1.5, 2, 1.7, 1.2, 1], 1.8, 1)
    cam = Cam([0.8, 1.3, 1.7, 2., 1.8, 1.2, 0.6], 3, 0.5)
    #gamma = 2.8
    xf, yf = 3, -2
    alpha_vec = np.arange(-2*np.pi, 4*np.pi + 0.05, 0.05)

    def update(val):
        ax.clear()
        cross_vec = []
        for alpha in alpha_vec:
            xa, ya = cam.r_cart(alpha, gamma_slider.val)
            xv, yv = cam.r_der_approx(alpha, gamma_slider.val)

            a = np.array([xa, ya, 0])
            v = np.array([xv, yv, 0])
            f = np.array([xf, yf,0])

            cross = np.cross(v, f-a)
            cross_vec.append(cross[2])

        ax.plot(alpha_vec, cross_vec)
        ax.plot([0,2*np.pi], [0,0], 'r-')

        al1 = bisection(cam, gamma_slider.val, xf, yf, -2*np.pi, 4*np.pi)
        al2 = naive_min(cam, gamma_slider.val, xf, yf)
        ax.plot([al1, al1], [-0.1,0.1], 'y-')
        ax.plot([al2, al2], [-0.1,0.1], 'g-')

        fig.canvas.draw_idle()

    gamma_slider.on_changed(update)
    plt.show()