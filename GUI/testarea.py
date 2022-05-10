# crank_and_rocker_motion.py

# import necessary packages
import numpy as np
from numpy import pi, radians, sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
from Fbarequations import FBarEquations

test = FBarEquations()

x,y = test.calculateLinks(450)
plt.plot(x,y)
plt.show()

