# -*- coding: utf-8 -*-

import board_detection as bd
import robot_detection as rd

import cv2
import numpy as np

if __name__ == "__main__":
    cap = cv2.VideoCapture(1)
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
        cv2.rectangle(img, rect_left, rect_right, color=(128, 128, 0), thickness=2)
        roi = img[rect_left[1]:rect_right[1], rect_left[0]:rect_right[0]]

        cv2.imshow("cropped_board", roi)

        cv2.imshow("find_board", img)

        center, r_image = rd.detect_robot_coords(roi)
        # tuning_color_filter(resized)
        cv2.imshow('robot', r_image)

        key = cv2.waitKey(1)
        # Выход из программы по нажатию esc
        if key == 27:
            exit()
