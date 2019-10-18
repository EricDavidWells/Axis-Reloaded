import threading
import queue
import time
import numpy as np
from MotionController import *
# from MotionController import MotionController

mode = 0
mc = MotionController()
mc.run()
mc.set_mode(2)
mc.set_correction(np.array([0, 10, 0]))

while True:
    try:
        mode = int(input("enter mode: "))
    except:
        pass

    mc.set_mode(mode)
    time.sleep(0.1)