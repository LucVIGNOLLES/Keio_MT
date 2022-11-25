#!/usr/bin/env python
import os.path
import numpy as np

# Define a filename.
filename = "cam.asc"

pt_list = []

if not os.path.isfile(filename):
    print('File does not exist.')
else:
    with open(filename) as f:
        for line in f.readlines():
            pt = [float(i) for i in line.split(" ")]
            pt_list.append(pt)

print(pt_list)

