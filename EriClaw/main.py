import threading
import queue
import time
import numpy as np
from MotionController import *
from VisionController import VisionController

def read_kbd_input(inputQueue):
    print('Input: ')
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
    input()
    # time.sleep(20)

    # Queue for 
    inputQueue = queue.Queue()
    print("Startup complete.")
    inputThread = threading.Thread(target=read_kbd_input, args=(inputQueue,), daemon=True)
    inputThread.start()

    while (True):
        correction = vc.getPosition()
        state = vc.getState()
        if state != "Centered":
            correction = np.array([correction[0], correction[1], 0])
            #print(correction)
            mc.set_correction(correction)
            mc.set_mode(2)
        else:

            mc.set_mode(1)

        if (inputQueue.qsize() > 0):
            input_str = inputQueue.get()
            #print("input_str = {}".format(input_str))

            if (input_str == EXIT_COMMAND):
                vc.shutDown()
                mc.set_mode(0)
                print("Press enter when plate is removed to complete shutdown.")
                input()
                mc.set_mode(3)
                print("Exiting serial terminal.")
                break
            else:
                if not input_str.isalpha():
                    mc.set_mode(int(input_str))
                # print(vc.getPosition())
                elif input_str == 's':
                    print(state + " at : " +str(correction[0]) +" " + str(correction[1]))
                elif input_str == 'recenter':
                    mc.set_mode(2)
                    mcState = mc.getMCState()
                    while mcState != 2:
                        print("Waiting to start recenter " )
                        time.sleep(.5)
                        mcState = mc.getMCState()



            # Insert your code here to do whatever you want with the input_str.

        # The rest of your program goes here.

        time.sleep(0.01) 
    print("End.")

if (__name__ == '__main__'): 
    main()