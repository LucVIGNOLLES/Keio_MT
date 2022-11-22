# p0 = np.array([equal_moment_x(0.08, h_fun(THETA0)), 0.08])
# t0 = (p0-S)/np.linalg.norm(p0-S)

# gamma = 0
# gamma_list = [gamma]

# p0p = rotate(p0, GAMMA_STEP)
# t0p = rotate(t0, GAMMA_STEP)

# y_array0 = np.arange(-0.1, 0.2, 0.01)
# x_array0 = [equal_moment_x(y, h_fun(THETA0)) for y in y_array0]

# y_array1 = np.arange(-0.1, 0.2, 0.01)
# x_array1 = [equal_moment_x(y, h_fun(THETA0 + THETA_STEP)) for y in y_array0]

# p1, t1, a = new_pt_tan_and_angle(p0p, t0p, h_fun(THETA0 + THETA_STEP))

# plt.plot(0,0.2, '.')

# plt.plot(x_array0, y_array0, linestyle = 'dashed')
# plt.plot(x_array1, y_array1, linestyle = 'dashed')

# plt.plot(p0[0], p0[1], '.')
# plt.plot([p0[0], p0[0] + t0[0]], [p0[1], p0[1] + t0[1]])

# plt.plot(p0p[0], p0p[1], '.')
# plt.plot([p0p[0], p0p[0] + t0p[0]], [p0p[1], p0p[1] + t0p[1]])

# plt.plot(p1[0], p1[1], '.')

# plt.grid()
# plt.show()     