import numpy as np
import matplotlib.pyplot as plt

def base_func(x):
    """
    Base function to use for the smooth step from 0 to 1.
    """
    return x**2

def norm_func(x):
    """
    Smooth step funciton created. Only valid form x=0 to x=1
    Must be resised and restricted to certain intervals
    """
    return (base_func(x))/(base_func(x) + base_func(1-x))

def blend_func(x, offset = 0, scale = 1):
    """
    Uses bits of the smooth step function to create a blending function, 
    centered on the offset variable and scaled according to the scale variable.
    Hre the function is made two pi periodical as well.
    """
    x = x - offset
    x = x%(2*np.pi)

    if x < 1*scale:
        return norm_func(-x/scale+1)
    elif 1*scale<= x < 2*np.pi-1*scale: 
        return 0
    elif 2*np.pi-1*scale <= x: 
        return norm_func((x-2*np.pi)/scale+1)
    else:
        return 0

