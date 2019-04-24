#!/usr/bin/env python
import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from marker_detector.msg import MarkerDetection
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from detector import Detector
from cam_feeder.msg import CamFeed


class MarkerDetector:
    def __init__(self):

        # Subscribes to the raspicam feed, every time a message is published
        # there our callback will activate
        self.subscriber = rospy.Subscriber('cam_feeder/image', CamFeed,
                                           self.callback)
        # make a publisher for the bounding boxes
        self.bb_pub = rospy.Publisher('marker_detector/detection',
                                      MarkerDetection, queue_size=1)

        self.bridge = CvBridge()

        self.detector = Detector(16)  # Dict 16 = original markers

    def callback(self, data):
        # Firstly, convert the image from topic to OpenCV
        array = np.fromstring(data.image, np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)

        ids, rects, img_out = self.detector.findMarkers(image)
        img_out = np.array(cv2.imencode('.jpg', img_out)[1]).tostring()

        frame = data.id

        if ids is not None:
            for iden, rect in zip(ids, rects):
                msg = self.construct_msg(iden, rect, frame)
                self.bb_pub.publish(msg)

    def construct_msg(self, iden, rect, frame):
        msg = MarkerDetection()
        msg.frame = frame
        msg.id = iden

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


