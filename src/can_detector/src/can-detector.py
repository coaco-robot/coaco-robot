#!/usr/bin/env python
import cv2
import rospy
import numpy as np

from detector import Detector
from can_detector.msg import CanDetection
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from cam_feeder.msg import CamFeed

class CanDetector:
    def __init__(self):

        # Subscribe to raspicam feed
        self.subscriber = rospy.Subscriber('cam_feeder/image', CamFeed, self.callback)
        
        # Create publisher to publish bounding boxes
        self.bb_pub = rospy.Publisher('can_detector/detection', CanDetection, queue_size=1)

        # Create publisher with detected frames
        self.out_pub = rospy.Publisher('can_detector/result', Image, queue_size=1)

        # Detector instance
        self.detector = Detector()

        # Image bridge instance
        self.bridge = CvBridge()


    def callback(self, data):
        # Convert image from topic to OpenCV
        cv_image = self.str2cv(data.image)
    
        output_frame, bb = self.detector.detect_can(cv_image)

        if bb:
            print("Found can")
            msg = self.construct_msg(data.image, output_frame, bb, data.id)
            self.bb_pub.publish(msg)


        # Publish output frame
        self.out_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


    def construct_msg(self, f_in, f_out, bb, frame_id):
        """ Construct message of type CanDetection """
        msg = CanDetection()
        msg.frame_id = frame_id
        msg.bb_x = bb[0]
        msg.bb_y = bb[1]
        msg.bb_width = bb[2]
        msg.bb_height = bb[3]

        return msg

    def str2cv(self, image_string):
        """ Convert image string to OpenCv image """
        np_arr = np.fromstring(image_string, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    
    def cv2str(self, cv_image):
        """ Convert OpenCv image to image string """
        return np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()


if __name__ == "__main__":
    rospy.init_node('can_detector')
    det = CanDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

