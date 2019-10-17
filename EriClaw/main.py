import threading
import queue
import time
import numpy as np
from MotionController import *
from VisionController import VisionController

def read_kbd_input(inputQueue):
    print('Ready for keyboard input:')
    while (True):
        input_str = input()
        inputQueue.put(input_str)

def main():
    EXIT_COMMAND = "exit"
    
    vc = VisionController()
    mc =  MotionController()
    # print("Initializing Motion Controller")
    mc.run()
    # print("Initializing Motion Controller")
    # print("Press enter to run VC")
    input()
    vc.run()

    # time.sleep(20)
    print("Startup complete.")
    # Queue for 
    inputQueue = queue.Queue()

    inputThread = threading.Thread(target=read_kbd_input, args=(inputQueue,), daemon=True)
    inputThread.start()

    while (True):
        correction = vc.getPosition()
        if not isinstance(correction,str):
            correction = np.array([correction[0], correction[1], 0])
            print(correction)
            mc.set_correction(correction)
            mc.set_mode(2)
        else:
            mc.set_mode(1)

        if (inputQueue.qsize() > 0):
            input_str = inputQueue.get()
            #print("input_str = {}".format(input_str))

            if (input_str == EXIT_COMMAND):
                vc.shutDown()
                print("Exiting serial terminal.")
                break
            else:
                mc.set_mode(int(input_str))
                print(vc.getPosition())

            # Insert your code here to do whatever you want with the input_str.

        # The rest of your program goes here.

        time.sleep(0.01) 
    print("End.")

if (__name__ == '__main__'): 
    main()