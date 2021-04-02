# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import math


def add(pt1, pt2):
    return pt1[0] + pt2[0], pt1[1] + pt2[1]


def to_int(pt):
    return tuple([int(x) for x in pt])


def find_color(img, lower_bound, upper_bound, draw_pos=False, show_mask=False,
               threshold=1):
    """
    Находит в изображении img объект на основе его цвета.

    Цвет задается границами lowerBound и upperBound.

    lowerBound2 и upperBound2 используются в случае, если искомый цвет - красный, т.к. он "обворачивается" вокруг HSV кольца.

    drawPos - рисует круг, где находится объект и показывает результируеще изображение.

    threshold - число пикселей искомого цвета, при котором считается, что объект есть в изображении.

    Если искомых пикселей меньше чем threshold, то возвращается (-1, -1).

    Иначе - кортеж из координат объекта.
    """

    # Перевод изображения в HSV
    img_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Создание бинарного изображения - маски, где пиксель имеет значение 1,
    # если на его месте в оригинальном изображении пискель цвета между upperBound и lowerBound
    mask = cv.inRange(img_HSV, lower_bound, upper_bound)

    # Создание моментов
    moments = cv.moments(mask)

    # Нахождение координат объекта
    if moments['m00'] < threshold:
        return -1, -1, np.zeros_like(mask)
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])

    if show_mask:
        cv.imshow("mask", mask)
        cv.waitKey(1)

    # if draw_pos:
    #     cv2.circle(img, (cx, cy), 10, (0, 0, 255), 3)
    #     cv2.imshow("img", img)
    #     cv2.waitKey(1)

    return cx, cy, mask


# a wrapper on cv2.connectedComponents
def conn_comps(mask, connectivity=8, left_edge=0, right_edge=np.inf):
    # mask = cv2.GaussianBlur(mask, (7, 7), 0, 0)
    num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(mask, connectivity)

    rects = []
    areas = []
    good_centroids = []

    for stat, centroid in zip(stats, centroids):
        # if stat[4] > MAX_AREA:
        #     MAX_AREA = stat[4]
        #     print(MAX_AREA)
        # else:
        #     continue

        if stat[2] == mask.shape[1] or stat[3] == mask.shape[0]:
            continue
        if centroid[0] < left_edge or centroid[0] > right_edge:
            continue
        cv.circle(mask, to_int(centroid), 3, (255, 255, 255))
        cv.putText(mask, str(stat[4]), to_int(centroid), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        rects.append(list(stat[:4]))
        areas.append(stat[4])
        good_centroids.append(to_int(centroid))

    zipped = zip(rects, areas, good_centroids)
    zipped = sorted(zipped, key=lambda t: -t[1])
    return list(zipped)


if __name__ == "__main__":
    img = cv.imread(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\test_photo_2.png")
    b_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    #ret, thresh = cv.threshold(img, 100, 255, type=cv.THRESH_BINARY + cv.THRESH_OTSU)
    thresh = cv.Canny(b_img, 0, 100, apertureSize=3)
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv.dilate(thresh, kernel)
    lines = cv.HoughLinesP(thresh, 1, np.pi / 180, 100, minLineLength=500, maxLineGap=30)
    transform = np.zeros(img.shape, dtype=np.uint8)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.line(transform, (x1, y1), (x2, y2), (255, 255, 255), 2)

    transform = cv.cvtColor(transform, cv.COLOR_BGR2GRAY)
    transform = cv.dilate(transform, kernel)
    transform = cv.GaussianBlur(transform, (5, 5), 3)
    transform = cv.dilate(transform, kernel, iterations=5)
    transform = cv.bitwise_not(transform)
    contours, hierarchy = cv.findContours(transform, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    cnt = contours[0]
    M = cv.moments(cnt)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, (0, 0, 255), 2)

    resize = cv.resize(thresh, (480, 480), interpolation=cv.INTER_AREA)
    resize2 = cv.resize(transform, (480, 480), interpolation=cv.INTER_AREA)
    resize1 = cv.resize(img, (480, 480), interpolation=cv.INTER_AREA)
    cv.imshow('img_r', resize1)
    cv.imshow('img', resize)
    cv.imshow('trans', resize2)
    k = cv.waitKey(0)
    if k == ord('q'):
        cv.destroyAllWindows()

    # cap = cv2.VideoCapture(1)
    # cap.set(5, 20)
    # c = 0
    # while True:
    #     # Считывание изображения с камеры
    #     a, img = cap.read()
    #
    #     sensitivity = 70
    #     lower_bound = np.array([0, 0, 255 - sensitivity])
    #     upper_bound = np.array([255, sensitivity, 255])
    #
    #     x, y, mask = find_color(img, lower_bound, upper_bound, draw_pos=True, show_mask=True, threshold=10000)
    #
    #     comp = conn_comps(mask)[0]
    #     # for comp in comps:
    #     rect = comp[0]
    #     print(f"Rect:{rect}, Length:{comp}")
    #     rect_left = (rect[0], rect[1])
    #     print(f"Left corner coord is: {rect_left[0]},{rect_left[1]}")
    #     rect_right = add(rect_left, (rect[2], rect[3]))
    #     print(f"Right corner coord is: {rect_right[0], rect_right[1]}")
    #     cv2.rectangle(img, rect_left, rect_right, color=(128, 128, 0), thickness=2)
    #     roi = img[rect_left[1]:rect_right[1], rect_left[0]:rect_right[0]]
    #
    #     cv2.imshow("cropped_board", roi)
    #
    #     cv2.imshow("find_board", img)
    #
    #     key = cv2.waitKey(1)
    #     # Выход из программы по нажатию esc
    #     if key == 27:
    #         exit()
