#! /usr/bin/env python
from __future__ import division

from collections import deque
from image_preprocessor import ImagePreprocessor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import rospy

QUEUE_LENGTH=10

NODE_NAME = "lane_detection_node"
IMG_SUB_TOPIC = "/camera/image_raw"
IMG_PUB_TOPIC = "/camera/lane"
class LaneDetector:
    def __init__(self, node_name, sub_topic, pub_topic):
        self.bridge = CvBridge()
        self.img_proc = ImagePreprocessor()

        # Publishers
        #self.

        rospy.init_node(node_name)

        # Crop parameters
        self.above = rospy.get_param("/asv_lane_track/lane_detection_node/above", 0.35)
        self.below = rospy.get_param("/asv_lane_track/lane_detection_node/below", 0.1)
        self.side = rospy.get_param("/asv_lane_track/lane_detection_node/side", 0)

        # Lane tracking parameters
        self.deviation = rospy.get_param("/asv_lane_track/lane_detection_node/deviation", 15)
        self.border = rospy.get_param("/asv_lane_track/lane_detection_node/border", 0)
        # Canny parameters
        self.low_thresh = rospy.get_param("/asv_lane_track/lane_detection_node/low_threshold", 50)
        self.high_thresh = rospy.get_param("/asv_lane_track/lane_detection_node/high_threshold", 150)
        self.aperture = rospy.get_param("/asv_lane_track/lane_detection_node/aperture", 3)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.img_callback)
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=5)

        self.yellow_masked_pub = rospy.Publisher("/yellow_masked", Image, queue_size=5)

        self.gray_pub = rospy.Publisher("/test", Image, queue_size=5)
        self.blur_pub = rospy.Publisher("/blur", Image, queue_size=5)

        self.left_lines = deque(maxlen=QUEUE_LENGTH)
        self.right_lines = deque(maxlen=QUEUE_LENGTH)
        rospy.spin()

    def process(self, img):
        cropped = self.img_proc.crop(img, self.above, self.below, self.side)
        mask = cropped
        yellow_mask = self.img_proc.select_yellow(mask)
        yellow_img = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)
        gray = self.img_proc.convert_gray(yellow_img)
        smooth_gray = self.img_proc.blur(gray, (self.deviation, self.deviation), self.border)
        edges = self.img_proc.edge_detection(smooth_gray, self.low_thresh, self.high_thresh, self.aperture)
        lines = self.img_proc.hough_lines(edges)
        # print(lines)

        return self.img_proc.draw_lines_from_points(cropped, lines)

        # left_line, right_line = self.img_proc.lane_lines(cropped, lines)
        #
        # def mean_line(line, lines):
        #     if line is not None:
        #         lines.append(line)
        #
        #     if len(lines) > 0:
        #         line = np.mean(lines, axis=0, dtype=np.int32)
        #         line = tuple(map(tuple, line))  # make sure it's tuples not numpy array for cv2.line to work
        #     return line
        #
        # left_line = mean_line(left_line, self.left_lines)
        # right_line = mean_line(right_line, self.right_lines)
        #
        # middle_line = ((int(left_line[0][0] + (right_line[0][0] - left_line[0][0]) / 2), int(left_line[0][1])),
        #                (int(left_line[1][0] + (right_line[1][0] - left_line[1][0]) / 2), int(left_line[1][1])))
        # return self.img_proc.draw_lane_lines(cropped, (left_line, right_line, middle_line))

    def img_callback(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        img_proc = cv_img
        # crop
        cropped = self.img_proc.crop(img_proc, self.above, self.below, self.side)

        # grayscale
        #gray =  self.img_proc.grayscale(cropped)

        # Extract yellow color in hsv
        yellow_masked = self.img_proc.select_yellow(cropped)
        res = cv2.bitwise_and(cropped,cropped, mask = yellow_masked)

        # blur
        # uhm why do you choose
        blurred = self.img_proc.blur(res, (self.deviation, self.deviation), self.border)

        # canny
        canny =  self.img_proc.edge_detection(blurred, self.low_thresh, self.high_thresh, self.aperture)
        # # cv2.imshow("canny", canny)
        # # cv2.waitKey(25)
        # #canny
        # lines = self.img_proc.hough_lines(canny)
        # left_line, right_line = self.img_proc.lane_lines(cv_img, lines)
        # final = self.img_proc.draw_lane_lines(cv_img, (left_line, right_line))
        # for line in lines:
        #     self.img_proc.draw_lane_lines()


        lane_img = self.process(cv_img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(lane_img, encoding="rgb8"))
            self.gray_pub.publish(self.bridge.cv2_to_imgmsg(canny))
            #cv2.imshow('mask', cv_img)
            # cv2.imshow('mask', res)
            #
            #cv2.waitKey(25)
            #self.yellow_masked_pub.publish(self.bridge.cv2_to_imgmsg(yellow_masked, encoding="rgb8"))

        except CvBridgeError as e:
            rospy.logerr(e)
        # height, width = canny.shape

        # Lane Detection



     # First perform IPM
        #self.

        #warped = cv_img

def main():
    try:
        LaneDetector(NODE_NAME, IMG_SUB_TOPIC, IMG_PUB_TOPIC)

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
if __name__ == "__main__":
    main()
