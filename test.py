import cv2
import numpy as np
import matplotlib.pyplot as plt


def nothing(x):
    pass


# read in image and save a version in hsv space
image = cv2.imread("pics/X3big_Y1smallR_north.jpg")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

h, s, v = cv2.split(hsv)

# this line here equalizes the images.
h, s, v = cv2.equalizeHist(h), cv2.equalizeHist(s), cv2.equalizeHist(v)

# sets up the track bars for our sliders
# cv2.namedWindow("imageRGB")
cv2.namedWindow("imageHSV")

cv2.createTrackbar('H_Low', 'imageHSV', 0, 255, nothing)
cv2.createTrackbar('H_High', 'imageHSV', 0, 255, nothing)
cv2.createTrackbar('S_Low', 'imageHSV', 0, 255, nothing)
cv2.createTrackbar('S_High', 'imageHSV', 0, 255, nothing)
cv2.createTrackbar('V_Low', 'imageHSV', 0, 255, nothing)
cv2.createTrackbar('V_High', 'imageHSV', 0, 255, nothing)

while (1):
    # get the values at the track bars for our threshold
    hlow = cv2.getTrackbarPos('H_Low', 'imageHSV')
    slow = cv2.getTrackbarPos('S_Low', 'imageHSV')
    vlow = cv2.getTrackbarPos('V_Low', 'imageHSV')

    hhigh = cv2.getTrackbarPos('H_High', 'imageHSV')
    shigh = cv2.getTrackbarPos('S_High', 'imageHSV')
    vhigh = cv2.getTrackbarPos('V_High', 'imageHSV')

    threshH = cv2.bitwise_not(cv2.inRange(h, hlow, hhigh))
    threshS = cv2.inRange(s, slow, shigh)
    threshV = cv2.inRange(v, vlow, vhigh)
    combined_total = threshH & threshV & threshS

    # stack the images so that we can see them in one window
    hsv = np.hstack((h, s, v))
    hsv_thresh = np.hstack((threshH, threshS, threshV))
    hsv_stack = np.vstack((hsv, hsv_thresh))

    final = np.hstack(combined_total)

    cv2.imshow('imageHSV', hsv_stack)
    cv2.imshow('combined', combined_total)
    k = cv2.waitKey(1) & 0xFF
