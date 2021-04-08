# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import math

from camera_vision.board_detection import find_board

# Подобранные трешхолды
H_MIN_UP = np.array((74, 32, 0), np.uint8)
H_MAX_UP = np.array((255, 255, 18), np.uint8)
H_MIN_DOWN = np.array((42, 0, 0), np.uint8)
H_MAX_DOWN = np.array((69, 255, 30), np.uint8)


def nothing(x):
    pass


def tuning_color_filter(test_img):
    """
    Вспомогательная функция для настройки порога цвета метки

    :param test_img: ndarray - image
    :return:
    """
    cv.namedWindow("result")
    cv.namedWindow("settings")
    cv.createTrackbar('h1', 'settings', 0, 255, nothing)
    cv.createTrackbar('s1', 'settings', 0, 255, nothing)
    cv.createTrackbar('v1', 'settings', 0, 255, nothing)
    cv.createTrackbar('h2', 'settings', 255, 255, nothing)
    cv.createTrackbar('s2', 'settings', 255, 255, nothing)
    cv.createTrackbar('v2', 'settings', 255, 255, nothing)

    while True:
        hsv = cv.cvtColor(test_img, cv.COLOR_BGR2HSV)

        h1 = cv.getTrackbarPos('h1', 'settings')
        s1 = cv.getTrackbarPos('s1', 'settings')
        v1 = cv.getTrackbarPos('v1', 'settings')
        h2 = cv.getTrackbarPos('h2', 'settings')
        s2 = cv.getTrackbarPos('s2', 'settings')
        v2 = cv.getTrackbarPos('v2', 'settings')

        h_min = np.array((h1, s1, v1), np.uint8)
        h_max = np.array((h2, s2, v2), np.uint8)

        # накладываем фильтр
        thresh = cv.inRange(hsv, h_min, h_max)

        cv.imshow('result', thresh)

        ch = cv.waitKey(1)
        if ch == ord('q'):
            break
    cv.destroyAllWindows()


def detect_label(img, h_min, h_max):
    """
    Детектирование метки на изображении вычисление по порогу цвета и основным моментам изображения.
    :param img: ndarray image
    :param h_min: (h, s, v) tuple of min HSV
    :param h_max: (h, s, v) tuple of max HSV
    :return: (x, y) - label's center coords
    """

    # To hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv = img
    thresh = cv.inRange(hsv, h_min, h_max)

    # Уберем лишнее и сгладим края опеннингом
    kernel = np.ones((5, 5), np.uint8)
    open_img = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
    cv.imshow(f'labels{h_max[0]}', open_img)

    # Вычислим моменты
    moments = cv.moments(open_img, 1)
    d_m01 = moments['m01']
    d_m10 = moments['m10']
    d_area = moments['m00']

    # Проверяем на корректность
    if d_area > 150:
        x = int(d_m10 / d_area)
        y = int(d_m01 / d_area)
        return (x, y)
    else:
        return None


def detect_robot_coords(img, resize=False):
    """
    Детектирование координат робота по фотографии и меткам.
    :param img: ndarray image
    :return: (x, y, z) координаты (x, y) ценра , поворот робота; img
    """

    cropped = find_board(img)  # TODO Добавить кроп доски после написания детектора доски
    if resize:
        # Уменьшаем картинку чтоб было видно на экране
        scale_percent = 20
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv.resize(cropped, dim, interpolation=cv.INTER_AREA)
    else:
        resized = cropped
    # Детектируем метки
    up_label = detect_label(resized, H_MIN_UP, H_MAX_UP)
    down_label = detect_label(resized, H_MIN_DOWN, H_MAX_DOWN)

    # Отрисовывем картинку
    cv.circle(resized, up_label, 5, (0, 255, 0), -1)
    cv.circle(resized, down_label, 5, (255, 0, 0), -1)
    cv.arrowedLine(resized, down_label, up_label, (0, 0, 255), 3)
    vector = (up_label[0] - down_label[0], up_label[1] - down_label[1])
    angle = vector[0] / (
        math.sqrt(vector[0] ** 2 + vector[1] ** 2))

    return up_label, resized, angle


if __name__ == '__main__':
    # test = cv.imread(r'/Users/valeriy/Downloads/test_spo.JPG')
    # detect_robot_coords(test)

    cap = cv.VideoCapture(0)
    # cv.namedWindow("result")
    # cv.namedWindow("settings")
    # cv.createTrackbar('h1', 'settings', 0, 255, nothing)
    # cv.createTrackbar('s1', 'settings', 0, 255, nothing)
    # cv.createTrackbar('v1', 'settings', 0, 255, nothing)
    # cv.createTrackbar('h2', 'settings', 255, 255, nothing)
    # cv.createTrackbar('s2', 'settings', 255, 255, nothing)
    # cv.createTrackbar('v2', 'settings', 255, 255, nothing)
    while True:
        ret, frame = cap.read()
        if not ret:
            print('not')
        cv.imshow('fr', frame)
        center, res, angle = detect_robot_coords(frame)
        cv.imshow('result', res)
        print(angle)
        # hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # h1 = cv.getTrackbarPos('h1', 'settings')
        # s1 = cv.getTrackbarPos('s1', 'settings')
        # v1 = cv.getTrackbarPos('v1', 'settings')
        # h2 = cv.getTrackbarPos('h2', 'settings')
        # s2 = cv.getTrackbarPos('s2', 'settings')
        # v2 = cv.getTrackbarPos('v2', 'settings')
        #
        # h_min = np.array((h1, s1, v1), np.uint8)
        # h_max = np.array((h2, s2, v2), np.uint8)
        #
        # # накладываем фильтр
        # thresh = cv.inRange(frame, h_min, h_max)

        # cv.imshow('result', thresh)
        k = cv.waitKey(1)
        if k == ord('q'):
            break
    cap.release()

    # center, image = detect_robot_coords(test)
    # img = test
    # scale_percent = 20
    # width = int(img.shape[1] * scale_percent / 100)
    # height = int(img.shape[0] * scale_percent / 100)
    # dim = (width, height)
    # cropped = img
    # resized = cv.resize(cropped, dim, interpolation=cv.INTER_AREA)
    # tuning_color_filter(resized)
    # cv.imshow('robot', image)
    # cv.waitKey(0)
