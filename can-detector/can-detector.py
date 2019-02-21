#!/usr/bin/python3
import cv2
import argparse
import numpy as np

print("Opencv version: {}".format(cv2.__version__))

parser = argparse.ArgumentParser(description="Detect soda cans in images")
parser.add_argument("image_path")
args = parser.parse_args()

# Open image file
orig = cv2.imread(args.image_path, help="Image to process")
img = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)

# Calculate mask with greyscale threshold
low = 0
high = 200
mask = cv2.inRange(img, low, high)

# Close the blobs
kernel = np.ones((5, 5), np.uint8)
mask = cv2.dilate(mask, kernel, iterations=1)
mask = cv2.erode(mask, kernel, iterations=1)

# Calculate the controus
_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the biggest contour
contour = max(contours, key=cv2.contourArea)

# Get the bounding rect for the biggest contour
x, y, w, h = cv2.boundingRect(contour)
cv2.rectangle(orig, (x, y), (x+w, y+h), (0, 255, 0), 2)

cv2.imshow("Can", orig)
cv2.waitKey(0)
