import numpy as np
import matplotlib.pyplot as plt

class Tsa:
    def __init__(self, len_d, rad_0, sep_a, sep_b, sprg_k=1) -> None:
        self.d = len_d # Twist zone lenght
        self.L0 = np.sqrt(len_d**2 + (sep_a + sep_b)**2) # Lenght when motor angle is 0
        self.r0 = rad_0 # Coiling radius
        self.sa = sep_a 
        self.sb = sep_b
        self.k = sprg_k
        self.theta_max = (np.pi/2*len_d)/rad_0 - (sep_a+sep_b)/rad_0
        #TODO: Check that theta_max is the same regardelss of sep_a and sep_b
        # it might be off by pi

    def h(self, theta):
        # Cap the contraction by taking max motor angle into account
        if theta > self.theta_max:
             return self.h(self.theta_max)
        # Special case whan both separators are non zero, we can extend the range to -pi
        elif -np.pi <= theta < 0 and self.sa > self.r0 and self.sa > self.r0:
            return self.L0 - np.sqrt(self.d**2 + self.sa**2 + self.sb**2 -2*self.sa*self.sb*np.cos(theta))
        # If theta is not valid, we return the base lenght of the actuator, with warning
        elif theta < 0:
            print("Warning: invalid angle for Tsa contraction")
            return self.h(0)
        # Nominal case
        else:
            return np.sqrt(self.d**2 + (self.sa+self.sb+self.r0*theta)**2) - self.L0

    def len_der(self, theta):
        """
        Needs to be rewrtitten if ever to be used
        """
        print("Tsa.len_der() hasn't been written yet dumbass")
        return 0

    def theta(self, h):
        """
        Needs to be cleaned to take special cases into account at the beginning of the pull
        """
        if (self.L0 + h)**2 - self.d**2 >= 0:
            return (-(self.sa+self.sb) + np.sqrt((self.L0 + h)**2 - self.d**2))/self.r0
        else:
            return self.L0

    def theta_rel(self, h, theta_0 = 0):
        """
        Simple method for setting the reference angle of the motor to something other than 0
        """
        return self.theta(h) - theta_0

    def reduction_ratio(self, theta):
        return self.d/(self.r0*(self.sa + self.sb + self.r0*theta))


# Testing =====

if __name__ == "__main__":
    actuator = Tsa(0.3, 0.006, 0.03, 0.03, 1)
    print(actuator.theta_max)

    theta_list = np.arange(0, actuator.theta_max + 10, 0.1)
    l_list = []
    ld_list = []
    t_list = []

    r_list = [actuator.reduction_ratio(theta) for theta in theta_list]

    for theta in theta_list:
        l_list.append(actuator.h(theta)- actuator.h(0))
        ld_list.append(actuator.len_der(theta))
        t_list.append(actuator.theta(actuator.h(theta)))

    plt.plot(theta_list, r_list)
    plt.plot(actuator.theta_max, 0, '.r')
    plt.plot()

    #plt.plot(theta_list/(2*np.pi), l_list)  
    #plt.plot(theta_list/(2*np.pi), ld_list)
    #plt.plot(theta_list, t_list, '.')
    plt.show()

