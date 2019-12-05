from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import math


low_H = 44 # 53
low_S = 144 # 162
low_V = 229 # 160
high_H = 108 # 140
high_S = 255 # 218
high_V = 255 # 242
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'


def return_cxp_cyp(izp, iyp):
    scale_factor = math.sqrt(307200/8000000)
    f = 3.04 * 10 ** -1
    size_pixel = 1.12 * 10 ** -4
    height = 13.5
    cxp = scale_factor * f * height / (size_pixel * (izp - 480 // 2))
    cyp = (iyp - 640 // 2) * cxp * size_pixel / (scale_factor * f)
    return cxp, cyp


parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv.VideoCapture(args.camera)
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)


# Setting up blob detection
params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 25
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False
detector = cv.SimpleBlobDetector_create(params)

while True:

    ret, frame = cap.read()
    if frame is None:
        break

    image = frame
    if image is None:
        break

    frame_HSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(frame_HSV)
    white = 255 * np.ones_like(h)
    threshH = white - cv.inRange(h, low_H, high_H)
    threshS = cv.inRange(s, low_S, high_S)
    threshV = cv.inRange(v, low_V, high_V)
    frame_threshold = threshH & threshS & threshV
    frame_threshold = cv.medianBlur(frame_threshold, 5)

    key_points = detector.detect(255 - frame_threshold)

    im_with_key_points = cv.drawKeypoints(frame_threshold, key_points, np.array([]), (0, 0, 255),
                                          cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    x = 0.0
    y = 0.0
    s = 1.0
    rows = im_with_key_points.shape[0]
    cols = im_with_key_points.shape[1]
    try:
        x = key_points[0].pt[0]
        y = key_points[0].pt[1]
        s = key_points[0].size
        checkForBlack = True
        while checkForBlack:
            y = y + 1
            checkForBlack = False
            for i in range(0, cols):
                if frame_threshold.item(int(y), int(i)) != 0:
                    checkForBlack = True
    except:
        print("")

    #print(int(x), int(y))
    print(rows)
    print_x = float(x)
    print_y = float(y)

    cv.circle(im_with_key_points, (int(x), int(y) - 1), 5, (0, 255, 0), -1)
    cv.imshow(window_capture_name, image)
    cv.imshow(window_detection_name, im_with_key_points)

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
