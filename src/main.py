import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer
from scipy.optimize import root
from math import cos, sin, pi
import serial
import lss
import lss_const as lssc
import time
import pickle
from scipy.interpolate import RegularGridInterpolator as rgi
from motioncontrol import *

mc = MotionController()
mc.run()

while True:
    mode = int(input("enter mode: "))
    mc.set_mode(mode)