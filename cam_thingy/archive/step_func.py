import numpy as np
import matplotlib.pyplot as plt

def base_func(x):
    return x**2

def norm_func(x):
    return (base_func(x))/(base_func(x) + base_func(1-x))

def step_func(x, offset = 0, scale = 1):
    x = x/scale
    offset = offset/scale
    if x < offset-1:
        return 0
    elif offset-1 <= x < offset: 
        return norm_func(x+1-offset)
    elif offset <= x < 1+offset: 
        return norm_func(-x+1+offset)
    else:
        return 0

X = np.arange(-1, 2, .05)[1:]
Y = []
for x in X:
    y = step_func(x, 1, 1/2)
    Y.append(y)

plt.plot(X, Y, '-')
plt.show()