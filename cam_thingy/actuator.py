import numpy as np
import matplotlib.pyplot as plt
from random import randint
from cam import Cam
from tsa import Tsa

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
        l1_0, _, _ = self.delta_l_out(gamma0, 1e-6, 0)

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

    def evaluate(self, gamma0, gammam, step_sz, goal_coeff):
        """
        The steps are compared to the goal steps calcualated from the goal coeff c as gamma = c*theta
        The returned value gets closer to 0 as the curve gets more linear
        """
        theta_lst, _ = self.run(gamma0, gammam, step_sz)

        # Evaluate using the list of corresponding theta angles for evenly spaced gamma positions
        deviation = 0
        goal_step = step_sz/goal_coeff

        for i in range(len(theta_lst)-1):
            deviation += (goal_step - abs(theta_lst[i+1] - theta_lst[i]))**2

        return deviation

## Testing ======

if __name__ == "__main__":
    tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
    actu = Actuator.randomCam(20, 3, 0.5, tsa,  0.15, -0.2)

    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        x, y = actu.cam.r_cart(theta, 0)
        X.append(x)
        Y.append(y)

    print(actu.cam.keypoints)

    plt.plot(X,Y)
    plt.plot(0,0, '.')
    plt.show()