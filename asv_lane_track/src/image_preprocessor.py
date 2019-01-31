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