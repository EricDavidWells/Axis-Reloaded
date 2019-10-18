# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import threading
import queue
import platform
from math import sin, cos, pi

class VisionController:
    # position = ''    
    shutdown = False
    def run(self):
        global shutdown
        global position
        global state
        state = ""
        shutdown = False
        position = ''
        print("Initializing Vision Controller")
        t = threading.Thread(target=self.run_thread)
        t.start()

    def getPosition(self):
        # return self.visionQueue.get()
        return position
    def getState(self):
        # return self.visionQueue.get()
        return state
    def shutDown(self):
        global shutdown
        shutdown = True

    def run_thread(self):
        global position
        global state
        if platform.system() == 'Windows':
            vs = VideoStream(src=1).start()
        else:
            vs = VideoStream(src=0).start()
        time.sleep(2.0)
        pts = deque(maxlen=64)
        greenLower = (69,53,26)
        greenUpper = (134,196,176)
        thresh = 254
        first = 1
        
        while True:
            if shutdown:
                break
            # grab the current frame
            frame = vs.read()

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                break

            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(frame, width=600)
            # print(frame.shape)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # cv2.imshow("gray",gray)

            # key = cv2.waitKey(0) & 0xFF

            blurred = cv2.GaussianBlur(gray, (11, 11), 0)
            blurred = cv2.erode(blurred, None, iterations=2)
            blurred = cv2.dilate(blurred,None, iterations=2)
            # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            (t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
            cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # mask = cv2.inRange(hsv, greenLower, greenUpper)
            # mask = cv2.erode(mask, None, iterations=2)
            # mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            # cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            #     cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            cv2.rectangle(frame,(250, 130), (350, 190),(255, 255, 0), 2)
            # only proceed if at least one contour was found
            while len(cnts) < 1:
                (t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
                cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                # mask = cv2.inRange(hsv, greenLower, greenUpper)
                # mask = cv2.erode(mask, None, iterations=2)
                # mask = cv2.dilate(mask, None, iterations=2)

                # find contours in the mask and initialize the current
                # (x, y) center of the ball
                # cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                #     cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                thresh = thresh - 1

                if thresh == 100:
                    thresh = 254
            
                (t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
                cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
            if first == 1:
                print("Vision Controller Ready. Press enter to continue.")
                first = 0
            (t, binary) = cv2.threshold(blurred, thresh - 10, 255, cv2.THRESH_BINARY)
            cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            if len(cnts) >= 1:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = min(cnts, key=lambda x:abs(cv2.contourArea(x) - 7300))
                # c = max(cnts, key=cv2.contourArea)
                # print(cv2.contourArea(c))
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                # Check if center is outside of 2 cm from middle
                # print(str((abs(x - 225)**2 + abs(y - 300)**2) **.5))
                xl = 12.5 / 48 * (x - 300)
                yl = 12.5 / 48 * (y - 225)
                theta = 36 * pi / 180
                xg = -xl * cos(theta) - yl * sin(theta)
                yg = -(-xl * sin(theta) + yl * cos(theta))
                if (abs(x - 300)**2 + abs(y - 225)**2) ** .5 >= 56.8:
                    # print("OFF CENTRE!")

                    # print(str(xg)+" " + str(yg))
                    position = (xg, yg)
                    state = "Off"
                else:
                    # print(str(x)+' '+str(y))
                    state = "Centered"
                    position = (xg, yg)
                # Should have a check for minimum radius being appropriate
                # 
            #     M = cv2.moments(c)
            #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
            #     # only proceed if the radius meets a minimum size
            #     if radius > 10:
            #         # draw the circle and centroid on the frame,
            #         # then update the list of tracked points
            #         cv2.circle(frame, (int(x), int(y)), int(radius),
            #             (0, 255, 255), 2)
            #         cv2.circle(frame, center, 5, (0, 0, 255), -1)

            #         if (x<250) or (x>350) or (y<130) or (y> 190):
            #             #print('Out of range!')
            #             # visionQueue.put('Out of range')
            #             position = 'Out of range'
            #         else:
            #             #print(str(x)+' '+str(y))
            #             # visionQueue.put(str(x)+' '+str(y))
            #             position = str(x)+' '+str(y)
                    

            # # update the points queue
            # pts.appendleft(center)

            # loop over the set of tracked points
            # for i in range(1, len(pts)):
            #     # if either of the tracked points are None, ignore
            #     # them
            #     if pts[i - 1] is None or pts[i] is None:
            #         continue

            #     # otherwise, compute the thickness of the line and
            #     # draw the connecting lines
            #     thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
            #     cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            # # show the frame to our screen
            cv2.imshow("Frame", binary)
            key = cv2.waitKey(1) & 0xFF

        vs.stop()
        cv2.destroyAllWindows()