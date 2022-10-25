import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

y = [1,3,5,7,2,4,9,6]
n = len(y)
x = range(0, n)

tck = interpolate.splrep(x, y, s=0)
xfit = np.arange(0, n-1, np.pi/50)
yfit = interpolate.splev(xfit, tck, der=0)

plt.plot(x, y, 'ro')
plt.plot(xfit, yfit,'b')
plt.plot(xfit, yfit)
plt.title("Spline interpolation In Python")
plt.show()