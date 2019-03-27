#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from marker_detector.msg import MarkerDetection
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from detector import Detector


class MarkerDetector:
    def __init__(self):

        # Subscribes to the raspicam feed, every time a message is published
        # there our callback will activate
        self.subscriber = rospy.Subscriber('raspicam_node/image/compressed',
                                           CompressedImage, self.callback)
        # make a publisher for the bounding boxes
        self.bb_pub = rospy.Publisher('marker_detector/detection',
                                      MarkerDetection, queue_size=1)

        self.bridge = CvBridge()

        self.detector = Detector(16)  # Dict 16 = original markers

    def callback(self, data):
        # Firstly, convert the image from topic to OpenCV
        array = np.fromstring(data.data, np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)

        ids, rects, img_out = self.detector.findMarkers(image)
        img_out = np.array(cv2.imencode('.jpg', img_out)[1]).tostring()

        frame = 0

	if ids is not None:
            msg = self.construct_msg(ids, rects, frame)
    	    self.bb_pub.publish(msg)

    def construct_msg(self, ids, rect, frame):
        msg = MarkerDetection()
        #msg.img_out = img_out
        msg.frame = frame
        msg.id = ids

        for x, y, w, h in rect:
            msg.bb_x.append(x)
            msg.bb_y.append(y)
            msg.bb_width.append(w)
            msg.bb_height.append(h)

        return msg


if __name__ == "__main__":
    rospy.init_node('marker_detector')
    det = MarkerDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


