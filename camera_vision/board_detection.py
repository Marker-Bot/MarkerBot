# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
from collections import defaultdict


def segment_by_angle_kmeans(lines, k=2, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle
    to segment `k` angles inside `lines`.
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get('attempts', 10)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented


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


def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def segmented_intersections(lines):
    """Finds the intersections between groups of lines."""

    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2))

    return intersections


def find_board(img, test=False, lenght=700, min_size=500, centroid=True):
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
    img = cv.imread(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\test_photo_3.png")
    find_board(img, test=True)

