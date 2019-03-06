#!/usr/bin/python3
import cv2
import argparse
import numpy as np

print("Opencv version: {}".format(cv2.__version__))

# Constants
h_low = 0
h_high = 139
s_low = 117
s_high = 255
v_low = 61
v_high = 255

resize = 10


def detect_can(frame):
    global  resize
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(img)

    # cv2.namedWindow("test")
    # cv2.createTrackbar('h_low', 'test', h_low, 255, update_h_low)
    # cv2.createTrackbar('h_high', 'test', h_high, 255, update_h_high)
    #
    # cv2.createTrackbar('s_low', 'test', s_low, 255, update_s_low)
    # cv2.createTrackbar('s_high', 'test', s_high, 255, update_s_high)
    #
    # cv2.createTrackbar('v_low', 'test', v_low, 255, update_v_low)
    # cv2.createTrackbar('v_high', 'test', v_high, 255, update_v_high)

    # while(1):
    #     h_mask1 = cv2.inRange(h, 0, h_low)
    #     h_mask2 = cv2.inRange(h, h_high, 255)
    #     h_mask = h_mask1 | h_mask2
    #
    #     s_mask = cv2.inRange(s, s_low, s_high)
    #     v_mask = cv2.inRange(v, v_low, v_high)
    #
    #     # mask = cv2.merge([h_mask, s_mask, v_mask])
    #     mask = h_mask & v_mask & s_mask
    #
    #     cv2.imshow("test", mask)
    #     cv2.waitKey(1)

    # Calculate the binary mask based on h, s and v
    h_mask = cv2.inRange(h, 0, h_low) | cv2.inRange(h, h_high, 255)
    s_mask = cv2.inRange(s, s_low, s_high)
    v_mask = cv2.inRange(v, v_low, v_high)

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

        # Make boundig box resize% larger
        resize = resize / 100

        x = int(x - (w * resize / 2))
        y = int(y - (h * resize / 2))
        w = int(w + (w * resize))
        h = int(h + (h * resize))

        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        bounding_box = (x, y, w, h)

    return frame, bounding_box


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detect soda cans in images")
    parser.add_argument("image_path", help="Image to process")
    args = parser.parse_args()

    # Open image file
    # orig = cv2.imread(args.image_path)
    cap = cv2.VideoCapture('video_small.mp4')

    counter = 0
    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        frame_out, bb = detect_can(frame)

        cv2.imshow("Can", frame_out)
        cv2.waitKey(3)
        counter += 1

    cap.release()
