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
        array = np.fromstring(data, np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)

        ids, rects, img_out = self.detector.findMarkers(image)
        img_out = np.array(cv2.imencode('.jpg', img_out)[1]).tostring()

        for identifier, rect in zip(ids, rects):
            msg = self.construct_msg(identifier, rect, img_out)
            self.bb_pub.publish(msg)

    def construct_msg(self, identifier, rect, img_out):
        msg = MarkerDetection()
        msg.img_out = img_out
        msg.id = identifier
        msg.bb_x = rect.x
        msg.bb_y = rect.y
        msg.bb_width = rect.width
        msg.bb_height = rect.height

        return msg


if __name__ == "__main__":
    rospy.init_node('marker_detector')
    det = MarkerDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


