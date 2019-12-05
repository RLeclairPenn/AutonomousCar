from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import math


class detectionClass:
    # this class represents what will be used for detection
    def __init__(self, lowh, lows, lowv, highh, highs, highv):
        self.low_H = lowh
        self.low_S = lows
        self.low_V = lowv
        self.high_H = highh
        self.high_S = highs
        self.high_V = highv
        self.window_capture_name = 'Video Capture'
        self.window_detection_name = 'Object Detection'
        self.low_H_name = 'Low H'
        self.low_S_name = 'Low S'
        self.low_V_name = 'Low V'
        self.high_H_name = 'High H'
        self.high_S_name = 'High S'
        self.high_V_name = 'High V'

        # This is to setup the camera
        self.parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
        self.parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
        self.args = self.parser.parse_args()
        self.cap = cv.VideoCapture(self.args.camera)

        # Setting up blob detection
        self.params = cv.SimpleBlobDetector_Params()
        self.params.filterByArea = True
        self.params.minArea = 25
        self.params.filterByCircularity = False
        self.params.filterByConvexity = False
        self.params.filterByInertia = False
        self.detector = cv.SimpleBlobDetector_create(self.params)

        # For do Analysis Test
        self.firstLoop = True


    def doAnalysisTest(self):
        if self.firstLoop:
            cv.namedWindow(self.window_capture_name)
            cv.namedWindow(self.window_detection_name)
            self.firstLoop = False
        ret, frame = self.cap.read()
        image = frame
        if image is None:
            return
        frame_HSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(frame_HSV)
        white = 255 * np.ones_like(h)
        threshH = white - cv.inRange(h, self.low_H, self.high_H)
        threshS = cv.inRange(s, self.low_S, self.high_S)
        threshV = cv.inRange(v, self.low_V, self.high_V)
        frame_threshold = threshH & threshS & threshV
        frame_threshold = cv.medianBlur(frame_threshold, 5)
        key_points = self.detector.detect(255 - frame_threshold)
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
            a = 0

        centery = rows / 2
        centerx = cols / 2
        iyp = abs(x - centerx)
        izp = abs(y - centery)

        cv.circle(im_with_key_points, (int(x), int(y) - 1), 5, (0, 255, 0), -1)
        cv.imshow(self.window_capture_name, image)
        cv.imshow(self.window_detection_name, im_with_key_points)

        key = cv.waitKey(30)
        if key == ord('q') or key == 27:
            return
        return iyp, izp

    def doAnalysis(self):
        ret, frame = self.cap.read()
        image = frame
        if image is None:
            return -1, -1
        frame_HSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(frame_HSV)
        white = 255 * np.ones_like(h)
        threshH = white - cv.inRange(h, self.low_H, self.high_H)
        threshS = cv.inRange(s, self.low_S, self.high_S)
        threshV = cv.inRange(v, self.low_V, self.high_V)
        frame_threshold = threshH & threshS & threshV
        frame_threshold = cv.medianBlur(frame_threshold, 5)
        key_points = self.detector.detect(255 - frame_threshold)
        if len(key_points) == 0:
            return -1, -1
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
            a = 0

        centery = rows / 2
        centerx = cols / 2
        iyp = abs(x - centerx)
        izp = abs(y - centery)

        key = cv.waitKey(30)
        if key == ord('q') or key == 27:
            return
        return iyp, izp


    def return_cxp_cyp(self, izp, iyp):
        scale_factor = math.sqrt(307200 / 8000000)
        f = 3.04 * 10 ** -1
        size_pixel = 1.12 * 10 ** -4
        height = 13.5
        cxp = scale_factor * f * height / (size_pixel * izp)
        cyp = iyp * cxp * size_pixel / (scale_factor * f)
        return cxp, cyp
