from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time

vs = VideoStream(src=1).start()

time.sleep(2.0)

thresh = 100

while True:
	frame = vs.read()

#	if frame is None:
#		break

	frame = imutils.resize(frame,width=600)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (11, 11), 0)
	blurred = cv2.erode(blurred, None, iterations=2)
	blurred = cv2.dilate(blurred,None, iterations=2)

	(t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
	cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	while len(cnts) != 1:
		frame = vs.read()

#	if frame is None:
#		break

		frame = imutils.resize(frame,width=600)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)
		blurred = cv2.erode(blurred, None, iterations=2)
		blurred = cv2.dilate(blurred,None, iterations=2)

		(t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
		cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		thresh = thresh + 1

		if thresh >= 255:
			thresh = 155
	 
		(t, binary) = cv2.threshold(blurred, thresh, 255, cv2.THRESH_BINARY)
		cnts = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	if len(cnts)> 0:
		for (i, c) in enumerate(cnts):
			print("\tSize of contour %d: %d" % (i, len(c)))
	cv2.drawContours(image = gray, contours = cnts, contourIdx = -1, color = (0, 0, 255), thickness = 5)
	cv2.imshow("Frame",gray)
	cv2.imshow("Binary",binary)
	key = cv2.waitKey(1) & 0xFF

	if key == ord("q"):
		break;
vs.stop()

cv2.destroyAllWindows()
