import camera_vision.board_detection as bd
import camera_vision.robot_detection as rd

import cv2 as cv
import numpy as np


def detect():
    cap = cv.VideoCapture(1)
    cap.set(5, 20)

    while True:
        # Считывание изображения с камеры
        a, img = cap.read()
        sensitivity = 60
        lower_bound = np.array([0, 0, 255 - sensitivity])
        upper_bound = np.array([255, sensitivity, 255])

        x, y, mask = bd.find_color(img, lower_bound, upper_bound, draw_pos=True, show_mask=True, threshold=10000)

        comp = bd.conn_comps(mask)[0]
        # for comp in comps:
        rect = comp[0]
        print(f"Rect:{rect}, Length:{comp}")
        rect_left = (rect[0], rect[1])
        print(f"Left corner coord is: {rect_left[0]},{rect_left[1]}")
        rect_right = bd.add(rect_left, (rect[2], rect[3]))
        print(f"Right corner coord is: {rect_right[0], rect_right[1]}")
        cv.rectangle(img, rect_left, rect_right, color=(128, 128, 0), thickness=2)
        roi = img[rect_left[1]:rect_right[1], rect_left[0]:rect_right[0]]

        cv.imshow("cropped_board", roi)

        cv.imshow("find_board", img)

        center, r_image = rd.detect_robot_coords(roi)
        # tuning_color_filter(resized)
        cv.imshow('robot', r_image)

        key = cv.waitKey(1)
        # Выход из программы по нажатию esc
        if key == 27:
            exit()


def test(img):
    sensitivity = 60
    lower_bound = np.array([0, 0, 255 - sensitivity])
    upper_bound = np.array([255, sensitivity, 255])

    x, y, mask = bd.find_color(img, lower_bound, upper_bound, draw_pos=True, show_mask=False, threshold=10000)

    comp = bd.conn_comps(mask)[0]
    # for comp in comps:
    rect = comp[0]
    print(f"Rect:{rect}, Length:{comp}")
    rect_left = (rect[0], rect[1])
    print(f"Left corner coord is: {rect_left[0]},{rect_left[1]}")
    rect_right = bd.add(rect_left, (rect[2], rect[3]))
    print(f"Right corner coord is: {rect_right[0], rect_right[1]}")
    cv.rectangle(img, rect_left, rect_right, color=(128, 128, 0), thickness=2)
    roi = img[rect_left[1]:rect_right[1], rect_left[0]:rect_right[0]]

    # cv.imshow("cropped_board", roi)

    cv.imshow("find_board", img)

    # center, r_image = rd.detect_robot_coords(roi)

    scale_percent = 40
    width = int(roi.shape[1] * scale_percent / 100)
    height = int(roi.shape[0] * scale_percent / 100)
    dim = (width, height)
    cropped = roi
    resized = cv.resize(cropped, dim, interpolation=cv.INTER_AREA)

    r_image = rd.tuning_color_filter(resized)
    # cv.imshow('robot', r_image)

    key = cv.waitKey(1)
    # Выход из программы по нажатию esc
    if key == 27:
        exit()


if __name__ == '__main__':
    test_im = cv.imread(r'/Users/valeriy/Downloads/test_spo.JPG')
    test(test_im)
    detect()