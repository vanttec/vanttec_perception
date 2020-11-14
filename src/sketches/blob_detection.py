# This is a PoC of RoboSub2021's target detection routine
# HSV values and selection crietria are specific to 2021 gameset

import cv2 as cv
import numpy as np
import sys

filename = sys.argv[1]

img = cv.imread(filename)

# Get lower half of image since non-circles are always there
# if img.shape[0] > 170:
#     img = img[int(img.shape[0]/2-20): img.shape[0], 0:img.shape[1]]

img = cv.GaussianBlur(img, (5,5),cv.BORDER_DEFAULT)

# we use two thresholds to cover red spectrum on the hue "wheel" 
first_lowrange = np.array([0,100,0])
first_highrange = np.array([10,255,255])

second_lowrange = np.array([150,100,0])
second_highrange = np.array([180, 255, 255])

hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
threshold = cv.inRange(hsv, first_lowrange, first_highrange)
threshold2 = cv.inRange(hsv, second_lowrange, second_highrange)
superposed = threshold2 + threshold

# blob detection parameters
params = cv.SimpleBlobDetector_Params()

params.minThreshold = 0;
params.maxThreshold = 255;

params.filterByArea = False
params.minArea = 0  
params.maxArea = 2000
params.filterByCircularity = False
params.minCircularity = 0.0
params.maxCircularity = 0.9

params.filterByConvexity = False
params.minConvexity = 0.5

params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
detector = cv.SimpleBlobDetector_create(params)


keypoints = detector.detect(superposed)

# we get the furthest center from origin which is guaranteed 
# to be non circular (provided it's not upside down)
target_center = [0,0]
for k in range(0,len(keypoints)):
    x = keypoints[k].pt[1]
    y = keypoints[k].pt[0]

    print(keypoints[k].pt[0])
    print(keypoints[k].pt[1])

    if x > target_center[1]:
        target_center[1] = int(x)
    if y > target_center[0]:
        target_center[0] = int(y)
print(target_center)

img = cv.circle(img, (target_center[0], 
    target_center[1]), 
    1,
    (0,255,0),
    5)

cv.imshow("Filtered", threshold)
cv.imshow("Filtered 2", threshold2)
cv.imshow("Superposed", superposed)
cv.imshow("Image", img)

cv.waitKey(0)
cv.destroyAllWindows()