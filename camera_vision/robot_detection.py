# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import math

from camera_vision.board_detection import find_board

# Подобранные трешхолды
H_MIN_UP = np.array((102, 255, 70), np.uint8)
H_MAX_UP = np.array((200, 255, 130), np.uint8)
H_MIN_DOWN = np.array((77, 179, 0), np.uint8)
H_MAX_DOWN = np.array((90, 255, 71), np.uint8)


def nothing(x):
    """
    Функция для теста

    :param x: -
    :return: -
    """
    pass


def tuning_color_filter(test_img):
    """
    Вспомогательная функция для настройки порога цвета метки.

    :param test_img: np.array - image
    :return:
    """

    cap = cv.VideoCapture(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\1.mp4")


    cv.namedWindow("result")
    cv.namedWindow("settings")
    cv.createTrackbar('h1', 'settings', 0, 255, nothing)
    cv.createTrackbar('s1', 'settings', 0, 255, nothing)
    cv.createTrackbar('v1', 'settings', 0, 255, nothing)
    cv.createTrackbar('h2', 'settings', 255, 255, nothing)
    cv.createTrackbar('s2', 'settings', 255, 255, nothing)
    cv.createTrackbar('v2', 'settings', 255, 255, nothing)

    while True:
        ret, test_img = cap.read()
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

        ch = cv.waitKey(25)
        if ch == ord('q'):
            break
    cv.destroyAllWindows()


def detect_label(img, h_min, h_max):
    """
    Детектирование метки на изображении вычисление по порогу цвета и основным моментам изображения.

    :param img: np.array image
    :param h_min: (h, s, v) кортеж мин. HSV трешхолдов
    :param h_max: (h, s, v) кортеж макс. HSV трешхолдов

    :return: (x, y) - Координаты центра метки
    """

    # To hsv
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #hsv = img
    thresh = cv.inRange(hsv, h_min, h_max)

    # Уберем лишнее и сгладим края опеннингом
    kernel = np.ones((5, 5), np.uint8)
    open_img = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
    #cv.imshow(f'labels{h_max[0]}', open_img)

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


def detect_robot_coords(img, previous_values, resize=False):
    """
    Детектирование координат робота по фотографии и меткам.

    :param img: np.array текущее изображение для детектирования
    :param previous_values: Результат предыдущего удачного детектирования
    :param resize: Изменение изображения в более удобный вид (для теста)

    :return: (x, y) координаты (x, y) ценра , img детекции,
    """

    cropped, flag = find_board(img)

    if not flag:
        return previous_values

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
    if up_label is None or down_label is None:
        return previous_values
    vector = (up_label[0] - down_label[0], up_label[1] - down_label[1])

    angle = vector[0] / (math.sqrt(vector[0] ** 2 + vector[1] ** 2))

    return up_label, resized, angle


if __name__ == '__main__':
    #test = cv.imread(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\test_photo_4.png")
    #cropped = find_board(test)
    #
    # _, thresh = cv.threshold(cropped, 127, 253, cv.THRESH_BINARY_INV)
    # cv.imshow("dkdk", thresh)
    # if cv.waitKey(0) == ord("q"):
    #     exit()
    #
    #tuning_color_filter(test)
    # detect_robot_coords(test)
    #exit()

    cap = cv.VideoCapture(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\2.mp4")
    previous_values = [None, None, None]

    ret, frame = cap.read()
    w, h, sz = frame.shape
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    video_writer_1 = cv.VideoWriter("detection2(1).avi", fourcc, 24.0, (800, 800))

    while True:
        ret, frame = cap.read()
        if not ret:
            print('not')
            break
        #cv.imshow('fr', frame)
        # k = cv.waitKey(1)
        # if k == ord('q'):
        #     break
        center, res, angle = detect_robot_coords(frame, previous_values)
        previous_values = [center, res, angle]
        if res is None:
            frame = cv.resize(frame, (800, 800), interpolation=cv.INTER_AREA)
            video_writer_1.write(frame)
        else:
            video_writer_1.write(res)




        #cv.imshow('result', res)
        #crop = find_board(frame)
        #cv.imshow("djjd", )

        # k = cv.waitKey(1)
        # if k == ord('q'):
        #     break
    cap.release()
    video_writer_1.release()


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
