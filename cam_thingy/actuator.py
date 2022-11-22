import numpy as np
import matplotlib.pyplot as plt
from random import randint, random
from cam import Cam
from tsa import Tsa

def bissect_lenght(a,b,alpha):
    return a*b/(a+b)*np.sqrt(2*(1+np.cos(2*alpha)))

class Actuator:
    def __init__(self, cam, tsa, x_sep, y_sep) -> None:
        self.cam = cam
        self.tsa = tsa
        self.xs = x_sep
        self.ys = y_sep

    @classmethod
    def randomCam(cls, num_keypoints, blend_range, perim, tsa, x_sep, y_sep):
        keypoints = []
        for i in range(num_keypoints):
            keypoints.append(randint(50, 100))
        cam = Cam(keypoints, blend_range, perim)

        return cls(cam, tsa, x_sep, y_sep)

    @classmethod
    def randomConvexCam(cls, num_iter, blend_range, perim_min, perim_max, tsa, x_sep, y_sep):
        keypoints = []
        for i in range(6):
            keypoints.append(randint(50, 80))

        for _ in range(num_iter):
            n = len(keypoints)
            new_kpts = []
            for i in range(n):
                new_kpts.append(keypoints[i])
                if i < n-1:
                    b = int(bissect_lenght(keypoints[i],keypoints[i+1],np.pi/n))
                else:
                    b = int(bissect_lenght(keypoints[n-1],keypoints[0],np.pi/n)) 
                new_kpts.append(randint(int(1.05*b), int(1.3*b)))
            keypoints = new_kpts
        
        cam = Cam(keypoints, blend_range, perim_min + random()*(perim_max - perim_min))
        return cls(cam, tsa, x_sep, y_sep)

    def __ge__(self, other):
        return True

    def __lt__(self, other):
        return True

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
            a = self.cam.r_cart(alpha, gamma)
            v = self.cam.approx_tangent(alpha, gamma)

            # Compute vectors
            a = np.array([a[0], a[1], 0])
            v = np.array([v[0], v[1], 0])
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

    def delta_l_out(self, gamma, thresh, alpha_min_old):
        """
        """
        s = np.array([self.xs, self.ys, 0])
        alpha_min = self.bisection(gamma, alpha_start = alpha_min_old, cross_thresh=thresh)

        # Point where the minimum was found (not always the right one)
        a_min = self.cam.r_cart(alpha_min, gamma)

        # Straight line between the contact point and the anchor.
        len1 = np.linalg.norm(np.array([s[0], s[1]]) - a_min) 

        # Note: This fix is cursed and will come haunt you and your loved ones one the happiest day of your life
        d_alpha = abs(alpha_min - alpha_min_old)
        if d_alpha > np.pi:
            d_alpha = 2*np.pi - d_alpha

        # Compare the position of the new contact point compared to the old one
        len2 = self.cam.approx_perim(alpha_min, d_alpha, 0.001)

        return len1, len2, alpha_min

    def run(self, gamma0, gammam, step_sz):
        """
        Compute a tab of gamma vs theta values to be evaluated for linearity
        """
        # Containers
        gamma_range = np.arange(gamma0, gammam, step_sz)
        theta_lst = []
        l_out = 0

        # Keep the initial lenght of string between the separator and the contact point on the cam
        # It will be subtracted from the other values so that we start at 0
        l1_0, _, _ = self.delta_l_out(gamma0, 1e-7, 0)

        # Find the contact point in the initial configuration
        alpha_min = self.bisection(gamma0)

        # For all configurations to be tested
        for gamma in gamma_range:
            # Get the lenghts and the contact point
            l1, delta_l2, alpha_min_new = self.delta_l_out(gamma, 1e-7, alpha_min)
            # Add the rolled lenght to keep track of the total
            l_out = l_out + delta_l2
            #print(delta_l2, "; ",l_out, "; ", alpha_min_new, "; ", alpha_min)
            alpha_min = alpha_min_new
            theta_lst.append(self.tsa.theta(l_out + l1 - l1_0))

        return theta_lst, gamma_range

    def run_force(self, gamma0, gammam, step_sz):
        """
        Compute a tab of gamma vs theta values to be evaluated for linearity
        """
        # Containers
        gamma_range = np.arange(gamma0, gammam, step_sz)
        theta_lst = []
        alpha_lst = []
        l_out = 0

        # Keep the initial lenght of string between the separator and the contact point on the cam
        # It will be subtracted from the other values so that we start at 0
        l1_0, _, _ = self.delta_l_out(gamma0, 1e-7, 0)

        # Find the contact point in the initial configuration
        alpha_min = self.bisection(gamma0)

        # For all configurations to be tested
        for gamma in gamma_range:
            alpha_lst.append(alpha_min)
            # Get the lenghts and the contact point
            l1, delta_l2, alpha_min_new = self.delta_l_out(gamma, 1e-7, alpha_min)
            # Add the rolled lenght to keep track of the total
            l_out = l_out + delta_l2
            #print(delta_l2, "; ",l_out, "; ", alpha_min_new, "; ", alpha_min)
            alpha_min = alpha_min_new
            theta_lst.append(self.tsa.theta(l_out + l1 - l1_0))

        return theta_lst, gamma_range, alpha_lst

    def evaluate(self, gamma0, gammam, step_sz, goal_coeff, return_lists = False):
        """
        First, the mean coeeficient is computed.
        Then, the individual steps are compared to the goal steps calcualated from the goal coeff c as gamma = c*theta
        The output is shaped so that the mean slope has a bigger priority
        The returned value gets closer to 0 as the curve gets more linear
        """
        theta_lst, gamma_lst = self.run(gamma0, gammam, step_sz)

        # Error on the mean coefficient
        coeff_error = abs((gammam - gamma0)/(theta_lst[-1] - theta_lst[0]) - goal_coeff)
        # Evaluate using the list of corresponding theta angles for evenly spaced gamma positions
        deviation = 0
        goal_step = step_sz/goal_coeff

        for i in range(len(theta_lst)-1):
            deviation += (goal_step - abs(theta_lst[i+1] - theta_lst[i]))**2

        if return_lists:
            return (coeff_error) + deviation/(100*coeff_error+1), gamma_lst, theta_lst
        else:
            return (coeff_error) + deviation/(10*coeff_error+1)

    def evaluate_force(self, gamma0, gammam, step_sz, goal_coeff, return_lists = False):
        theta_lst, gamma_lst, alpha_lst = self.run_force(gamma0, gammam, step_sz)
        reduc_ratio_lst = [self.tsa.reduction_ratio(theta) for theta in theta_lst]
        lever_arm_lst = [self.cam.lever_arm(alpha) for alpha in alpha_lst]

        error_lst = [(l*h-goal_coeff)**2 for l,h in zip(lever_arm_lst, reduc_ratio_lst)]

        if return_lists:
            return np.sum(error_lst), gamma_lst, [l*h for l,h in zip(lever_arm_lst, reduc_ratio_lst)]
        else:
            return np.sum(error_lst)

def plot_actu(actu, step, score):
    plt.figure(step)
    plt.xlabel(str(score))
    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        a = actu.cam.r_cart(theta, 0)
        X.append(a[0])
        Y.append(a[1])

    plt.plot(X,Y)
    plt.plot(0,0, '.')

    alpha = np.pi/4

    x, y = actu.cam.r_cart(alpha,0)
    dx, dy = actu.cam.approx_tangent(alpha, 0)

    xk, yk = [], []

    for i, r in enumerate(actu.cam.keypoints):
        xk.append(r * np.cos(i*actu.cam.step))
        yk.append(r*np.sin(i*actu.cam.step))
    xk.append(actu.cam.keypoints[0])
    yk.append(0)
    plt.plot(xk, yk, 'g.-')

    plt.plot([x, x+dx], [y, y+dy], '-')
    plt.savefig('figs/step' + str(step) + '.png')

## Testing ======

if __name__ == "__main__":
    tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
    cam = Cam([0.01567108, 0.01228622, 0.01430292, 0.01708767, 0.02457278, 0.03650728, 0.04674677, 0.05588914, 0.05855479, 0.05288107, 0.03709499, 0.02514716], 2, 0.22942268972602545)
    
    actu = Actuator(cam, tsa, 0.2, -0.15)

    t, g, a = actu.run_force(0, 2*np.pi, 0.1)
    plt.plot(t, a)
    plt.show()


    #TODO : find an upper limit for the new points raduis that depends on the current step angle 
    # The narrower the angle, the lower the limit should be 

    # doing it approximately might be good enought depending on the evolutive strategy we choose
    # e.g., at each generation, survivors are mutated by sequantially selecting a few keypoints to modify
    # If that keypoint is short compared to its neighbours, it is mutated to get longer
    # Else it is mutated to get shorter (slightly in each case)

    # This way the cam should stay pretty convex at is evolves.