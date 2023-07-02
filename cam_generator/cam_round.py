import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, root_scalar
from math import sqrt, sin, cos

D = 0.200
S = np.array([0.0605, 0.3625 - D]) # Coordinates of the string separator

def rotate(u, angle):
    """
    For a 2D vector, rotates it by a given angle in the direct direction
    """
    return np.array([u[0]*np.cos(angle) - u[1]*np.sin(angle), u[0]*np.sin(angle) + u[1]*np.cos(angle)])

# Cam points placement functions

def first_pt(s, radius):
    """
    Returns the tangent point to the separator line
    """

    res = root_scalar(lambda b0: s[0]*np.cos(b0) + s[1]*np.sin(b0) - radius, bracket=[-np.pi/4,np.pi/4]) # find the absolute angle to the point
    beta = res.root

    return np.array([radius*np.cos(beta), radius*np.sin(beta)])

def generate_round_cam(radius, s, num_pts, rot_range):
    """
    Sequencially generate a cam's outline, round  this  time.
    """

    p0 = first_pt(s, radius)

    step = rot_range/num_pts
    cam_pts = [rotate(p0, -i*step) for i in range(num_pts)]

    return cam_pts

class Cam:
    def __init__(self, s, radius, num_pts, range):
        self.pts = generate_round_cam(radius, s, num_pts, range)
        self.s_pos = s
        self.radius = radius
        self.num_pts = num_pts

    def ptsRotated(self, angle):
        """
        Returns a version of the outline points rotated by an angle gamma in the direct direction
        """
        return [rotate(pt, angle) for pt in self.pts]

    def perim(self, idx1, idx2):
        """
        Simple perim approximation. Returns the sum of all lenght of segments between two specified indexes
        """
        assert idx1 <= idx2, "idx1 should be greater than idx2"

        perim = 0

        for i in range(idx1, idx2-1):
            perim = perim + np.linalg.norm(self.pts[i+1] - self.pts[i])

        return perim

    def outStringLen(self, pt_idx):
        assert pt_idx < self.num_pts, "pt_idx out of range"

        perim = self.perim(pt_idx, self.num_pts - 1)
        pts = self.ptsRotated(pt_idx*self.alpha_step)
        line = np.linalg.norm(pts[pt_idx] - self.s_pos)

        return perim + line
    
    def show(self):
        fig, ax = plt.subplots()

        ax.plot(0,0,'ro', label = "Cam COR")
        ax.plot(self.s_pos[0], self.s_pos[1], 'go')

        ax.plot([pp[0] for pp in self.pts], [pp[1] for pp in self.pts], '.-', label = "Cam outline", linewidth=2)
        ax.grid()
        ax.set_aspect('equal')
        ax.set_xlabel("x coordinate (m)")
        ax.set_ylabel("y coordinate (m)")
        plt.legend()
        plt.show()

if __name__ == "__main__":
    cam = Cam(S, 0.04, 100, 200*np.pi/180)
    cam.show()

    filename = "cams/cam_round004.csv"

    with open(filename, 'w') as f:
        for p in cam.pts:
            f.write(str(1000*p[0]) + ',' + str(1000*p[1]) + ',' + str(0) + '\n') # 1000 foactor converts to mm