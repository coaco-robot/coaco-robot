#!/usr/bin/env python
import cv2
import rospy
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from cam_feeder.msg import CamFeed

class CamFeeder:
    def __init__(self):

        # Subscribe to raspicam feed
        self.cam_subscriber = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, self.cam_callback)
        #self.term_subscriber = rospy.Subscriber('seek/image/compressed', CompressedImage, self.term_callback)

        # Create publisher with detected frames
        self.cam_pub = rospy.Publisher('cam_feeder/image', CamFeed, queue_size=1)
        self.seek_pub = rospy.Publisher('cam_feeder/seek', CamFeed, queue_size=1)

        # List
        self.list = []
        self.current_id_cam = 0
        self.counter_cam = 0

        self.current_id_term = 0
        self.counter_term = 0

    def term_callback(self, data):
        if self.counter_term == 10:
            self.counter_term = 0

            # Convert image from topic to OpenCV
            cv_image = self.str2cv(data.data)

            # Republish with id
            msg = self.construct_msg(img, self.current_id_term)
            self.current_id_term += 1
            self.seek_pub.publish(msg)
        
        else:
            self.counter_cam += 1


    def cam_callback(self, data):
        if self.counter_cam == 10:
            self.counter_cam = 0

            # Convert image from topic to OpenCV
            cv_image = self.str2cv(data.data)

            # Resize
            img = cv2.resize(cv_image, (cv_image.shape[1]/2, cv_image.shape[0]/2))

            # Republish with id
            msg = self.construct_msg(img, self.current_id_cam)
            self.current_id_cam += 1
            self.cam_pub.publish(msg)
        
        else:
            self.counter_cam += 1


    def construct_msg(self, image, img_id):
        """ Construct message of type CamFeed """
        msg = CamFeed()
        msg.image = self.cv2str(image)
        msg.id = img_id

        return msg

    def str2cv(self, image_string):
        """ Convert image string to OpenCv image """
        np_arr = np.fromstring(image_string, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    
    def cv2str(self, cv_image):
        """ Convert OpenCv image to image string """
        return np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()


if __name__ == "__main__":
    rospy.init_node('cam_feeder')
    det = CamFeeder()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

