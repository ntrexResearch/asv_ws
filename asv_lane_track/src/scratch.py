import cv2
import numpy as np


def define_roi(img, above=0.0, below=0.0, side=0.0):
    height, width, channels = img.shape
    color_black = (0, 0, 0)
    print(height - int(height*below))
    img[height - int((height*below)):height, :] = color_black
    print("dd")
    pts = np.array([[0,0], [0, int(height*(above+0.15))], [int(width*side), int(height*above)], [width-int(width*side), int(height*above)], [width, int(height*(above+0.15))], [width, 0]], np.int32)
    print("test")
    cv2.fillPoly(img, [pts], color_black)
    print('pk')
    cv2.imshow("test", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()