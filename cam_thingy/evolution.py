import numpy as np
import matplotlib.pyplot as plt
from random import randint
from cam_gen import Cam
from Tsa import Tsa

class Actuator:
    def __init__(self, cam, tsa, x_sep, y_sep) -> None:
        self.cam = cam
        self.tsa = tsa
        self.xs = x_sep
        self.ys = y_sep

    def bisection(self, gamma, alpha_min = -2*np.pi, alpha_max = 4*np.pi, alpha_start = np.pi/4, cross_thresh = 1e-5, max_step = 1000):
        """
        Returns the relative angle for which the string contacts the cam
        Results are still a bit jagged, it might be best to look into why
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
            xa, ya = self.cam.r_cart(alpha, gamma)
            xv, yv = self.cam.r_der_approx(alpha, gamma)

            # Compute vectors
            a = np.array([xa, ya, 0])
            v = np.array([xv, yv, 0])
            f = np.array([self.xs, self.ys,0])

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

        #print(">>", steps)

        return alpha % (2*np.pi)

    def l_out(self, gamma, thresh):
        """
        Computes the lenght of cable that would be necessary to go from a specific point 
        on the cam to the anchored point. 
        """
        s = np.array([self.xs, self.ys, 0])
        alpha_min = self.bisection(gamma, cross_thresh=thresh)

        # Point where the minimum was found (not always the right one)
        xa_min, ya_min = self.cam.r_cart(alpha_min, gamma)

        # Straight line between the contact point and the anchor.
        len1 = np.linalg.norm(np.array([s[0], s[1]]) - np.array([xa_min, ya_min])) 
        #Perimeter on the cam between the contact point and an arbitrary point.
        len2 = self.cam.r_int_approx(alpha_min, np.pi, gamma, .09)

        return len1 + len2

    def l_out_rel(self, gamma, thresh = 1e-6):
        return self.l_out(gamma, thresh) - self.l_out(0, thresh)

    def delta_l_out(self, gamma, thresh, alpha_min_old):
        """
        """
        s = np.array([self.xs, self.ys, 0])
        alpha_min = self.bisection(gamma, alpha_start = alpha_min_old, cross_thresh=thresh)

        # Point where the minimum was found (not always the right one)
        xa_min, ya_min = self.cam.r_cart(alpha_min, gamma)

        # Straight line between the contact point and the anchor.
        len1 = np.linalg.norm(np.array([s[0], s[1]]) - np.array([xa_min, ya_min])) 

        # Note: This fix is cursed and will come haunt you and your loved ones one the happiest day of your life
        d_alpha = abs(alpha_min - alpha_min_old)
        if d_alpha > np.pi:
            d_alpha = 2*np.pi - d_alpha

        # Compare the position of the new contact point compared to the old one
        len2 = self.cam.get_perim(alpha_min, d_alpha, 0.001)

        return len1, len2, alpha_min

class Individual:
    def __init__(self, perimeter, blend_factor, num_keypoints random = True) -> None:
        self.perim = perimeter
        self.blend = blend_factor
        
        keypts = []

        for i in range(num_keypoints):
            keypts.append(randint(0,100))

        

        self.actu = Actuator()

        self.gamma_lst = []
        self.theta_lst = []

    def run(self, gamma0, gammam, step_sz):
        """
        Should return a tab of gamma vs theta values to be evaluated for linearity
        """

        # Create gamma range (mayeb as argument)
        # Keep track of lenght_out of Tsa, starting at zero
        # Only use delta_l_out and add it to that sum
        # feed that sum to the Tsa's inverse kenematics funciton to find Theta
        # ???
        # Profit

        gamma_range = np.arange(gamma0, gammam, step_sz)
        theta_lst = []
        alpha_lst = []
        l_lst = []
        l_out = 0

        l1_0, _, _ = self.actu.delta_l_out(gamma0, 1e-6, 0)

        alpha_min = self.actu.bisection(gamma0)

        for gamma in gamma_range:
            l1, delta_l2, alpha_min_new = self.actu.delta_l_out(gamma, 1e-7, alpha_min)
            l_out = l_out + delta_l2
            #print(delta_l2, "; ",l_out, "; ", alpha_min_new, "; ", alpha_min)
            alpha_min = alpha_min_new
            alpha_lst.append(alpha_min)
            l_lst.append(delta_l2)

            theta_lst.append(self.actu.tsa.theta(l_out + l1 - l1_0))

        self.gamma_lst = gamma_range
        self.theta_lst = theta_lst 
            
        return gamma_range, np.array(theta_lst), alpha_lst, l_lst

    def value(self, goal_coeff):
        """
        Value that should be minimimal when the thate gamma relation is the most lienar
        """

        deviation = 0
        h = abs(self.gamma_lst[1] - self.gamma_lst[0])

        goal_step = h/goal_coeff

        for i in range(len(self.theta_lst)-1):
            deviation += (goal_step - abs(self.theta_lst[i+1] - self.theta_lst[i]))**2

        return deviation


# TODO: Get rid of the iNdIviDuAl class, and turn all of its components into a single "evaluate(actuator)" function.
# Add an option to generate an actuator with random values for the variables
# Code the evolutive methods in the Population class


class Population:
    def __init__(self, population_sz) -> None:
        self.pop_size = population_sz
        self.individuals = []
        
        for i in range(population_sz):
            

    def mate():
        return 0

    def evolve():
        return 0

# Testing ======

GAMMA0 = 0
GAMMAM = np.pi

if __name__ == "__main__":
    cam = Cam([0.8, 1.1, 1.3, 3, 2, 1.7, 1.6], 1.8, 0.3)
    tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
    actu = Actuator(cam, tsa, 0.15, -0.2)

    indiv = Individual(actu)

    gamma_lst, theta_lst, alpha_lst, l_lst = indiv.run(GAMMA0, GAMMAM, 0.02)

    print(indiv.value(np.pi/92))

    plt.figure(1)
    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        x, y = cam.r_cart(theta, 0)
        X.append(x)
        Y.append(y)

    plt.plot(X,Y)
    plt.plot(0,0, '.')

    plt.figure(2)
    plt.plot(gamma_lst, theta_lst)
    # plt.plot(gamma_lst, alpha_lst)
    # plt.plot(gamma_lst, l_lst)
    plt.plot([GAMMA0, GAMMAM], [indiv.actu.tsa.theta_max, indiv.actu.tsa.theta_max], 'r-')
    plt.show() 

    