import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from modulo_func import blend_func

def norm2(u,v):
    return np.sqrt((v[0] - u[0])**2 + (v[1] - u[1])**2)

class Cam:
    def __init__(self, keypoint_radii, keypoint_range = 1, des_perim = 1) -> None:
        # First, fill the arguments with unscaled values
        self.keypoints = np.array(keypoint_radii) 
        self.step = 2*np.pi/len(keypoint_radii)
        self.range = keypoint_range

        # Compute the perimeter of the unscaled cam
        unscaled_perim = self.r_int_approx(0, 2*np.pi, 0)
        # Compute the nessecary scaling to apply in order for the cam to have the desired perimeter
        scale = des_perim/unscaled_perim

        # Apply scaling 
        self.keypoints = np.array(keypoint_radii)*scale

    def blend_custom(self, angle, k, range):
        """
        Uses the blend function to determine the summinf coefficient for each keypoint of the cam at a given angle.
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

    def r_cart(self, angle, gamma):
        """
        Returns the coordinates of a point at angle alpha on a cam in configuuration gamma, 
        assuming the center of rotation is the origin
        """
        return self.r(angle)*(np.cos(angle)*np.cos(gamma) - np.sin(angle)*np.sin(gamma)), self.r(angle)*(np.cos(angle)*np.sin(gamma) + np.sin(angle)*np.cos(gamma))

    def r_der_approx(self, alpha, gamma, h = 0.00000000001):
        """
        Simple approximation of the tangent to the cam by selecting two points defined by a very close alpha angle
        """
        x, y = self.r_cart(alpha, gamma) # pt1
        x_h, y_h = self.r_cart(alpha+h, gamma) # pt2

        return (x_h - x)/h, (y_h - y)/h
    
    def r_int_approx(self,angle1, angle2, gamma, h = .01):
        """
        Returns the approximate arc lenght along a cam in position gamma for angles between alpha1 and alpha2 
        """
        alpha_vec = np.arange(angle1, angle2+h, h)[1:]
        sum = 0
        for alpha in alpha_vec:
            pt1 = self.r_cart(alpha, gamma)
            pt2 = self.r_cart(alpha+h, gamma)

            sum += norm2(pt1, pt2)
        
        return sum

    #TODO: we might not need to use gamma when computing tangents or arc lenghts. 
    # but doing it without using absolute coordinates seems complicated anyway.


## Testing ======

if __name__ == "__main__":
    test_cam = Cam([1, 1, 1.5, 3, 1.7, 1.2, 1], 2.5, 2)

    perim1 = test_cam.r_int_approx(0, np.pi*2, 0)

    print(perim1)

    theta_vec = np.arange(0, 2 * np.pi+0.05, .02)[1:]
    X = []
    Y = []
    for theta in theta_vec:
        x, y = test_cam.r_cart(theta, 0)
        X.append(x)
        Y.append(y)

    n = len(test_cam.keypoints)

    for i, r in enumerate(test_cam.keypoints):
        
        x, y = r * np.cos(i*test_cam.step), r*np.sin(i*test_cam.step)
        plt.plot(x, y, 'g.')

    plt.plot(X, Y, '-')
    plt.plot(0,0, 'r.')
    plt.show()