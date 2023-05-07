import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from cam_gen import Cam, D, A, B, R0

theta0 = 5*2*np.pi
thetam = 25*2*np.pi

def tsaLen(theta, theta0):
    return sqrt(D**2 + (A+B + R0*theta)**2) - sqrt(D**2 + (A+B + R0*theta0)**2)

xi = 40
cam1 = Cam(np.array([0.1, 0.2]), theta0, thetam, 500, xi, 0)
cam1.show()
l0 = cam1.outStringLen(0)

theta_step = cam1.xi*cam1.alpha_step
theta_list = [theta0+i*theta_step for i in range(cam1.num_pts)]

len_in_lst = [tsaLen(theta, theta0) for theta in theta_list]
len_out_lst_cam1 = [cam1.outStringLen(i) - l0 for i in range(cam1.num_pts)]

len_total_lst = [lin + lout for lin, lout in zip(len_in_lst, len_out_lst_cam1)]

#plt.plot(theta_list, len_out_lst_cam1, label = 'lout')
#plt.plot(theta_list, len_in_lst, label = 'lin')
plt.plot(theta_list, len_total_lst, label = 'tot')
plt.legend()
plt.show()




# def epsilon_solve_fun_2(s, p, t, h, xi):
#     """
#     Equation that needs to be solved to find the next point for the cam. 
#     The solution epsilon is how much the points need to be moved on the current tangent line 
#     of the cam.
#     """
#     return lambda e:  h*((p[0]+e*t[0])*(-s[0]+(p[0]+e*t[0])) - (p[1]+e*t[1])*(-s[1]+(p[1] + e*t[1]))) - xi*sqrt((-s[0]+(p[0]+e*t[0]))**2 + (-s[1]+(p[1]+e*t[1]))**2)

# def moment_point(s, theta, xi):
#     h = h_fun(theta) # F/T
#     r = xi/h #r0*h = Cam torque / motor torque = xi

#     res = root_scalar(lambda b0: s[0]*np.cos(b0) + s[1]*np.sin(b0) - r, bracket=[-np.pi/4,np.pi/4])
#     beta = res.root

#     return np.array([r*np.cos(beta), r*np.sin(beta)])
    
# eps = np.arange(-0.1,0.1, 0.001)

# p0 = moment_normal_point(S, 0, XI)
# t0 = (p0-S)/np.linalg.norm(p0-S)
# tgt_line = [p0+e*t0 for e in eps]

# p = rotate(p0, np.pi/4)
# t = rotate(t0, np.pi/4)
# tgt_line_rt = [p+e*t for e in eps]

# A = moment_point(S,0.1*XI, XI)

# plt.plot(0,0,'ro')
# plt.plot(S[0], S[1], 'go')
# plt.plot(p0[0], p0[1], 'y.')
# plt.plot(p[0], p[1], 'b.')
# plt.plot([pt[0] for pt in tgt_line], [pt[1] for pt in tgt_line], 'y-')
# plt.plot([pt[0] for pt in tgt_line_rt], [pt[1] for pt in tgt_line_rt], 'b-')

# plt.plot([A[0], S[0]], [A[1], S[1]], '-')

# teps = np.arange(-1,1, 0.01)
# eq_moment_line = [A + t*(S-A) for t in teps]
# plt.plot([pt[0] for pt in eq_moment_line], [pt[1] for pt in eq_moment_line], 'g-')

# u_int = ((S[0] - p[0])*(S[1] - A[1]) - (S[1] - p[1])*(S[0] - A[0]))/((S[0] - A[0])*(-t[1]) - (S[1] - A[1])*(-t[0]))

# p_new = p+u_int*t

# plt.plot(p_new[0], p_new[1], 'or')

# plt.grid()
# plt.show()
