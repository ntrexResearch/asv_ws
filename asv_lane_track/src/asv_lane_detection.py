#! /usr/bin/env python

from image_preprocessor import ImagePreprocessor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

NODE_NAME = "lane_detection_node"
IMG_SUB_TOPIC = "image"

class LaneDetector:
    def __init__(self, node_name, sub_topic):
        self.bridge = CvBridge()
        self.img_proc = ImagePreprocessor()

        # Publishers
        #self.

        rospy.init_node(NODE_NAME)

        # Crop parameters
        self.above = rospy.get_param("/asv_lane_track/lane_detection_node/above", 0.58)
        self.below = rospy.get_param("/asv_lane_track/lane_detection_node/below", 0.1)
        self.side = rospy.get_param("/asv_lane_track/lane_detection_node/side", 0.3)

        # Lane tracking parameters
        self.deviation = rospy.get_param("/asv_lane_track/lane_detection_node/deviation", 5)
        self.border = rospy.get_param("/asv_lane_track/lane_detection_node/border", 0)
        # Canny parameters
        self.low_thresh = rospy.get_param("/asv_lane_track/lane_detection_node/low_threshold", 50)
        self.high_thresh = rospy.get_param("/asv_lane_track/lane_detection_node/high_threshold", 150)
        self.aperture = rospy.get_param("/asv_lane_track/lane_detection_node/aperture", 3)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.img_callback)

        rospy.spin()

    def img_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # First perform IPM
        #self.

        warped = cv_img
        # crop
        cropped = self.img_proc.crop(warped, self.above, self.below, self.side)

        # grayscale
        gray =  self.img_proc.grayscale(cropped)

        # blur
        # uhm why do you choose
        blurred = self.img_proc.blur(gray, (self.deviation, self.deviation), self.border)

        # canny
        canny =  self.img_proc