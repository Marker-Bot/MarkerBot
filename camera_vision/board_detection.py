# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from collections import defaultdict


def find_board(img, test=False, lenght=700, min_size=1300, centroid=True):
    b_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    thresh = cv.Canny(b_img, 0, 100, apertureSize=3)
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv.dilate(thresh, kernel, iterations=3)

    lines = cv.HoughLinesP(thresh, 1, np.pi / 180, 100, minLineLength=lenght, maxLineGap=30)
    transform = np.zeros(img.shape, dtype=np.uint8)

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if test:
            cv.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.line(transform, (x1, y1), (x2, y2), (255, 255, 255), 2)

    transform = cv.cvtColor(transform, cv.COLOR_BGR2GRAY)
    transform = cv.dilate(transform, kernel, iterations=7)
    transform = cv.bitwise_not(transform)

    counters, heir = cv.findContours(transform, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    squares = []
    for cnt in counters:
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.04 * peri, True)

        if len(approx) == 4 and cv.contourArea(cnt) > min_size:
            M = cv.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            squares.append((cnt, (cx, cy)))

    nearest_square = squares[0]
    if centroid:
        img_center = (img.shape[0] // 2, img.shape[1] // 2)
        for square in squares:
            if abs(square[1][0] - img_center[1]) < abs(nearest_square[1][0] - img_center[1]) and abs(
                    square[1][1] - img_center[0]) < abs(nearest_square[1][1] - img_center[0]):
                nearest_square = square

    peri = cv.arcLength(nearest_square[0], True)
    approx = cv.approxPolyDP(nearest_square[0], 0.04 * peri, True)
    if test:
        for a in approx:
            cv.circle(img, (a[0][0], a[0][1]), 5, (255, 0, 0), -1)

    pts1 = np.float32([approx[0][0], approx[1][0], approx[2][0], approx[3][0]])
    pts2 = np.float32([[0, 0], [0, 800], [800, 800], [800, 0]])

    M = cv.getPerspectiveTransform(pts1, pts2)

    perspective = cv.warpPerspective(img, M, (800, 800))
    if test:
        cv.imshow('perspective', perspective)
        resize = cv.resize(thresh, (480, 480), interpolation=cv.INTER_AREA)
        resize2 = cv.resize(transform, (480, 480), interpolation=cv.INTER_AREA)
        resize1 = cv.resize(img, (480, 480), interpolation=cv.INTER_AREA)
        cv.imshow('img_r', resize1)
        cv.imshow('img', resize)
        cv.imshow('trans', resize2)
        k = cv.waitKey(0)
        if k == ord('q'):
            cv.destroyAllWindows()

    return perspective


if __name__ == "__main__":
    img = cv.imread(r"/Users/valeriy/Downloads/test_spo.JPG")
    find_board(img, test=True)
