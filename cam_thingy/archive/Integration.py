import numpy as np
from tsa import Tsa
from cam import Cam
from archive.cam_contact import bisection, naive_min
import matplotlib.pyplot as plt

# Anchor point's position ralative to the cam's center of rotation
XS = 0.15
YS = -0.2

# Desired movement range of the driven arm 
GAMMA0 = 0
GAMMAM = np.pi

def norm2(u,v):
    """
    Returns the distance defined by norm2 between to vectors
    """
    return np.sqrt((v[0] - u[0])**2 + (v[1] - u[1])**2)

def l_cam(cam, xs, ys, gamma, thresh):
    """
    Computes the lenght of cable that would be necessary to go from a specific point 
    on the cam to the anchored point. 
    """
    s = np.array([xs, ys, 0])
    alpha_min = bisection(cam, gamma, xs, ys, cross_thresh=thresh)

    # Point where the minimum was found (not always the right one)
    xa_min, ya_min = cam.r_cart(alpha_min, gamma)

    # Straight line between the contact point and the anchor.
    len1 = norm2((s[0], s[1]), (xa_min, ya_min)) 
    #Perimeter on the cam between the contact point and an arbitrary point.
    len2 = cam.r_int_approx(alpha_min, np.pi, gamma)

    return len1 + len2

def l_cam_naive(cam, xs, ys, gamma):
    """
    Computes the lenght of cable that would be necessary to go from a specific point 
    on the cam to the anchored point. 
    """

    s = np.array([xs, ys, 0])
    alpha_min = naive_min(cam, gamma, xs, ys)

    # Point where the minimum was found (not always the right one)
    xa_min, ya_min = cam.r_cart(alpha_min, gamma)

    # Straight line between the contact point and the anchor.
    len1 = norm2((s[0], s[1]), (xa_min, ya_min)) 
    # Perimeter on the cam between the contact point and an arbitrary point.
    len2 = cam.r_int_approx(alpha_min, np.pi, gamma)

    return len1 + len2

def l_rel_cam(cam, xs, ys, gamma, thresh):
    """
    Returns how mush string needs to be given of taken from the cam part for it to rotate from gamma = 0 to a given angle
    """
    return l_cam(cam, xs, ys, gamma, thresh) - l_cam(cam, xs, ys, 0, thresh)

def l_rel_naive(cam, xs, ys, gamma):
    """
    Returns how mush string needs to be given of taken from the cam part for it to rotate from gamma = 0 to a given angle
    """
    return l_cam_naive(cam, xs, ys, gamma) - l_cam_naive(cam, xs, ys, 0)

# Main ======

if __name__ == "__main__":

    # Define cam part and actuator part
    cam = Cam([0.8, 1.3, 1.7, 2., 1.8, 1.2, 0.6], 3, 0.5)
    actu = Tsa(0.2, 0.003, 0.01, 0, 1)

    # Define angle restrictions for the rotation of the driven arm (and direction as well)
    gamma_range = np.arange(GAMMAM, GAMMA0, -0.04)
    theta_list1 = []
    theta_list2 = []

    l_list1 = []
    l_list2 = []

    for gamma in gamma_range:
        # TODO: It'd be more efficient to compute the change in lenght between each pass (ie, only the "first" and last segments in l_cam)
        # Here we're calling l_cam twice for no reason in l_rel_cam.
        l1 = l_rel_cam(cam, XS, YS, gamma, 1e-7)
        l2 = l_rel_cam(cam, XS, YS, gamma, 1e-8)


        l_list1.append(l1)
        l_list2.append(l2)
        theta_list1.append(actu.theta_rel(l1))
        theta_list2.append(actu.theta_rel(l2)) 

    plt.figure(1) # Angle relation graph

    plt.plot([GAMMA0, GAMMAM], [actu.theta_max, actu.theta_max], 'r-')
    plt.plot(gamma_range, theta_list1, 'b.')
    plt.plot(gamma_range, theta_list2, 'g.')

    plt.figure(2) # Cam shape

    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        x, y = cam.r_cart(theta, 0)
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.plot(0,0, '.')
    plt.show()


    