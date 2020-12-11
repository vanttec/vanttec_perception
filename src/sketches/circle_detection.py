# This is a PoC of RoboSub2021's target detection routine

import numpy as np
import cv2 as cv
import sys

file_name = sys.argv[1]
print(file_name)
img = cv.imread(file_name, 0)
# img = cv.medianBlur(img,5)
if (img.shape[0] > 300):
    img = img[0:int(img.shape[0]/2), 0:img.shape[1]]

print("Image height: " + str(img.shape[0]))

cimg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, 2000,
                          param1=50, param2=20, minRadius=10, maxRadius=70)
try:
    circles = np.uint16(np.around(circles))

    for c in circles[0, :]:
        # draw the outer circle
        cv.circle(cimg, (c[0], c[1]), c[2], (0, 255, 0), 2)
        # draw the center of the circle
        cv.circle(cimg, (c[0], c[1]), 2, (0, 0, 255), 3)
        cv.imshow('detected circles', cimg)
except:
    print("Circle not found")
finally:

    cv.waitKey(0)
    cv.destroyAllWindows()
