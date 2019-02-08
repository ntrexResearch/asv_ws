#! /usr/bin/env python

import cv2
import numpy as np

class ImagePreprocessor:
    def define_roi(self, img, above=0.0, below=0.0, side=0.0):
        height, width, channels = img.shape
        color_black = (0, 0, 0)
        img[height - int((height*below)):height, :] = color_black
        pts = np.array([[0, 0], [0, int(height * (above + 0.15))], [int(width * side), int(height * above)],
                        [width - int(width * side), int(height * above)], [width, int(height * (above + 0.15))],
                        [width, 0]], np.int32)
        cv2.fillPoly(img,[pts], color_black)
        return img


    def crop(self, img, above=0.0, below=0.0, side=0.0):
        height, width, channels = img.shape
        img = img[int(height*above) : height - int((height*below)), int((width*side)):width - int((width*side))]
        return img

    def grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    def blur(self, img, kernel_size, border):
        return cv2.GaussianBlur(img, kernel_size, border)


    def edge_detection(self, img, thresh1, thresh2, aperture):
        return cv2.Canny(img, thresh1, thresh2, aperture)


    def warp_perspective(self, img, transformation_matrix, resolution):
        return cv2.warpPerspective(img, transformation_matrix, resolution)

    def filter_color(self, img, lower, upper):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array(lower, np.uint8)
        upper = np.array(upper, np.uint8)
        color_mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=color_mask)

    def convert_hsv(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    def convert_hls(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

    def convert_gray(self, img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def select_yellow(self, img):
        converted_hsv = self.convert_hsv(img)
        #print(hue_mask.shape)
        #yellow mask
        lower = (80, 30, 100) #cv2.cv.Scalar(20, 100, 100)
        upper = (100, 200, 255) #cv2.cv.Scalar(30, 255, 255)
        #lower = np.uint8([20, 100, 100])
        #upper = np.uint8([30, 255, 255])
        yellow_mask = cv2.inRange(converted_hsv, lower, upper)
        # gray = cv2.cvtColor(yellow_mask, cv2.COLOR_HSV2GRAY)
        #cv2.imwrite("~/yellow_mas.png",yellow_mask)
        #print(yellow_mask.shape)
        # mask = cv2.bitwise_or(white_mask, yellow_mask)
        #return yellow_mask
        return yellow_mask # cv2.bitwise_and(img, img, mask=gray)


    def hough_lines(self, img):
        return cv2.HoughLinesP(img, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)


    def average_slope_intercept(self, lines):
        left_lines = [] #(slope, intercept)
        left_weights = [] # length
        right_lines = []
        right_weights = []
        if lines is None:
            return None, None
        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope*x1
                length = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))

                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))
        left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        return left_lane, right_lane #in shape (slope, intercept), (slope, intercept)

    def make_line_points(self, y1, y2, line):
        if line is None:
            return None
        slope, intercept = line

        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))

    def lane_lines(self, img, lines):
        left_lane, right_lane = self.average_slope_intercept(lines)

        y1 = img.shape[0]
        y2 = y1 * 0.4

        left_line = self.make_line_points(y1, y2, left_lane)
        right_line = self.make_line_points(y1, y2, right_lane)

        return left_line, right_line

    def draw_lane_lines(self, img, lines, color=[255, 0, 0], thickness = 20):
        line_image = np.zeros_like(img)
        for line in lines:
            if line is not None:
                cv2.line(line_image, line[0], line[1], color, thickness)
        return cv2.addWeighted(img, 1.0, line_image, 0.95, 0.0)

    def draw_lines_from_points(self, img, lines, color=[255, 0, 0], thickness = 20):
        line_image = np.zeros_like(img)
        for line in lines:
            if line is not None:
                cv2.line(line_image, (line[0][0], line[0][1]), (line[0][2], line[0][3]), color, thickness)
        return cv2.addWeighted(img, 1.0, line_image, 0.95, 0.0)
