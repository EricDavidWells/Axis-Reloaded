# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
import threading
import queue

class VisionController:
    position = ''    
    shutdown = False
    def run(self):
        global shutdown
        shutdown = False
        t = threading.Thread(target=self.run_thread)
        t.start()

    def getPosition(self):
        # return self.visionQueue.get()
        return position

    def shutDown(self):
        global shutdown
        shutdown = True

    def run_thread(self):
        global position
        vs = VideoStream(src=0).start()
        time.sleep(2.0)
        pts = deque(maxlen=64)
        greenLower = (69,53,26)
        greenUpper = (134,196,176)
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
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # construct a mask for the color "green", then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            mask = cv2.inRange(hsv, greenLower, greenUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            cv2.rectangle(frame,(250, 130), (350, 190),(255, 255, 0), 2)
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    if (x<250) or (x>350) or (y<130) or (y> 190):
                        #print('Out of range!')
                        # visionQueue.put('Out of range')
                        position = 'Out of range'
                    else:
                        #print(str(x)+' '+str(y))
                        # visionQueue.put(str(x)+' '+str(y))
                        position = str(x)+' '+str(y)
                    

            # update the points queue
            pts.appendleft(center)

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

        vs.stop()
        cv2.destroyAllWindows()