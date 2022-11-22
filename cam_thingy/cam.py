import numpy as np
import matplotlib.pyplot as plt
from blend_keypoints import blend_func

class Cam:
    def __init__(self, keypoint_radii, keypoint_range = 1, des_perim = 1) -> None:
        # First, fill the arguments with unscaled values
        self.keypoints = np.array(keypoint_radii, float) 
        self.step = 2*np.pi/len(keypoint_radii)
        self.range = keypoint_range
        self.perim = des_perim

        # Compute the perimeter of the unscaled cam
        unscaled_perim = self.approx_perim(0, 2*np.pi - 0.005)
        # Compute the nessecary scaling to apply in order for the cam to have the desired perimeter
        scale = des_perim/unscaled_perim

        # Apply scaling 
        self.keypoints = np.array(keypoint_radii)*scale

    def blend_custom(self, angle, k, range):
        """
        Uses the blend function to determine the summing coefficient for each keypoint of the cam at a given angle.
        The range variable allows to choose how far each keypoint can affect its neighbors
        """
        angle = angle%(2*np.pi) # Bring angle back in the [0, 2pi] interval
        return 1/range*blend_func(angle, k*self.step, range*self.step)

    def blend_vector(self, angle):
        """
        Computes the weights for all the values of the cam, return a vector of them
        """
        # TODO: Might be more efficient only to compute the weights that are non zero only.
        f = np.zeros_like(self.keypoints) # Container for keypoints weights
        for k in range(len(f)): 
            f[k] = self.blend_custom(angle, k, self.range)
    
        return f

    def r(self, angle):
        """
        Elementwise multiplication of weights and values, summed to obtain the final radius at a given angle
        """
        return np.sum(self.keypoints * self.blend_vector(angle))

    def r_dot(self, angle, h):
        return (self.r(angle+h) - self.r(angle))/h

    def r_cart(self, angle, gamma):
        """
        Returns the coordinates of a point at angle alpha on a cam in configuuration gamma, 
        assuming the center of rotation is the origin
        """
        return np.array([self.r(angle)*np.cos(angle+gamma), self.r(angle)*np.sin(angle+gamma)])

    def approx_tangent(self, alpha, gamma, h = 0.00000000001):
        """
        Simple approximation of the tangent to the cam by selecting two points defined by a very close alpha angle
        """
        a = self.r_cart(alpha, gamma) # pt1
        a_h = self.r_cart(alpha+h, gamma) # pt2

        return (a_h - a)/h

    def approx_perim(self, alpha, delta_alpha, h = 0.005):
        """
        Returns the arc lenght on the cam between alpha1 and alpha + delta_alpha. 
        The lenght if signed according to the direct rotation direction
        """
        alpha_vec = np.arange(alpha, alpha + delta_alpha, h)[1:]
        sum = []
        for alpha in alpha_vec:
            pt1 = self.r_cart(alpha, 0)
            pt2 = self.r_cart(alpha+h, 0)

            sum.append(np.linalg.norm(pt1 - pt2))

        return np.sum(sum)

    def lever_arm(self, alpha):
        # pure derivatire of r(alpha) is  needed for this computation.
        # Might be more efficicent to know the analytic expression
        return (self.r(alpha)**2)/np.sqrt(self.r_dot(alpha, 1e-9)**2 + self.r(alpha)**2)

## Testing ======

if __name__ == "__main__":

    amp = 0.2
    angle = 0
    offset = 2
    scale = 2

    X = np.arange(0,2*np.pi, 0.1)
    Y = [amp*blend_func(x, offset, scale) + 1 for x in X]

    plt.plot(X,Y)
    plt.show()

    # test_cam = Cam([1,1,1,1], 2, 0.5)

    # h = 1e-12

    # theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    # X = []
    # Y = []

    # der = []
    # der_n = []
    # R = []
    # r_n = []
    # for theta in theta_vec:
    #     x, y = test_cam.r_cart(theta, 0)
    #     X.append(x)
    #     Y.append(y)

    #     r = test_cam.r(theta)
    #     rh = test_cam.r(theta+h)

    #     dr = (rh - r)/h

    #     dr_n = dr/np.sqrt(dr**2 + r**2)

    #     R.append(r)
    #     r_n.append(r/np.sqrt(dr**2 + r**2))
    #     der_n.append(dr_n)
    #     der.append(dr)


    # n = len(test_cam.keypoints)

    # plt.figure(1)

    # xk, yk = [], []

    # for i, r in enumerate(test_cam.keypoints):
    #     xk.append(r * np.cos(i*test_cam.step))
    #     yk.append(r*np.sin(i*test_cam.step))
    # xk.append(test_cam.keypoints[0])
    # yk.append(0)
    # plt.plot(xk, yk, 'g.-')

    # plt.plot(X, Y, '-')
    # plt.plot(0,0, 'r.')

    # plt.figure(2)
    # plt.plot(theta_vec, r_n, 'b-')
    # plt.plot(theta_vec, der_n, 'r-')
    # for i, r in enumerate(test_cam.keypoints):
    #     plt.plot(i*test_cam.step, 0, '.g')
    # plt.show()