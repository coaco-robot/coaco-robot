#!/usr/bin/env python
import cv2
import numpy as np

class Detector:
    def __init__(self):
        # Constants
	self._H_LOW = 0
	self._H_HIGH = 139
	self._S_LOW = 117
	self._S_HIGH = 255
        self._V_LOW = 61
    	self._V_HIGH = 255
    	self._RESIZE = 10

    def detect_can(self, frame):
	img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(img)
	
	# Calculate the binary mask based on h, s and v
	h_mask = cv2.inRange(h, 0, self._H_LOW) | cv2.inRange(h, self._H_HIGH, 255)
	s_mask = cv2.inRange(s, self._S_LOW, self._S_HIGH)
	v_mask = cv2.inRange(v, self._V_LOW, self._V_HIGH)
	
	# Combine masks
	mask = h_mask & v_mask & s_mask
	
	# Close the blobs
	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.dilate(mask, kernel, iterations=1)
	mask = cv2.erode(mask, kernel, iterations=1)
	
	# Calculate the controus
	_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	bounding_box = False
	
	# Find the biggest contour
	if contours:
	    contour = max(contours, key=cv2.contourArea)
	
	    # Get the bounding rect for the biggest contour
	    x, y, w, h = cv2.boundingRect(contour)
	
	    # Make boundig box RESIZE% larger
	    resize = self._RESIZE / 100
	
	    x = int(x - (w * resize / 2))
	    y = int(y - (h * resize / 2))
	    w = int(w + (w * resize))
	    h = int(h + (h * resize))
	
	    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
	
	    bounding_box = (x, y, w, h)
	
	return frame, bounding_box
