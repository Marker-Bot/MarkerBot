# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np


def nothing(x):
    """
    Функция для теста

    :param x: -
    :return: -
    """
    pass


def find_board(img: np.array, test=False, lenght=883, min_size=527132, max_size=1500000, centroid=True, record=False):
    """
    Функция для поиска доски на изображении

    :param img: RGB изображение np.array
    :param test: bool настройка параметров
    :param lenght: int Минимальная длинна линии доски
    :param min_size: int Минимальный размер определившейся доски
    :param max_size: int Максимальный размер определившейся доски
    :param centroid: bool Центрирована ли доска на frame
    :param record: запись проходных изображений

    :return: img, flag - img - перспективное преобразование доски ( кроп доски)
             flag - True - преобразование выполнено; False - преобразование не выполнено, img - оригинальная картинка
    """
    if not test:
        th_can_down = 60
        th_can_up = 69
        th_houg = 135
        min_leght = lenght
        gap = 128
        min_size = min_size
    else:
        th_can_down = cv.getTrackbarPos('th_can_down', 'settings')
        th_can_up = cv.getTrackbarPos('th_can_up', 'settings')
        th_houg = cv.getTrackbarPos('th_hough', 'settings')
        min_leght = cv.getTrackbarPos('min_line', 'settings')
        gap = cv.getTrackbarPos('gap', 'settings')
        min_size = cv.getTrackbarPos('min_size', 'settings')

    b_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    thresh = cv.Canny(b_img, th_can_down, th_can_up, apertureSize=3)
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv.dilate(thresh, kernel, iterations=4)

    if test:
        resize = cv.resize(thresh, (480, 480), interpolation=cv.INTER_AREA)
        resize1 = cv.resize(img, (480, 480), interpolation=cv.INTER_AREA)
        cv.imshow('img_r', resize1)
        cv.imshow('canny', resize)
        k = cv.waitKey(1)
        if k == ord('q'):
            cv.destroyAllWindows()

    lines = cv.HoughLinesP(thresh, 1, np.pi / 180, th_houg, minLineLength=min_leght, maxLineGap=gap)
    transform = np.zeros(img.shape, dtype=np.uint8)

    if lines is None:
        if record:
            return [img], False
        return img, False

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if test:
            cv.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.line(transform, (x1, y1), (x2, y2), (255, 255, 255), 2)

    transform = cv.cvtColor(transform, cv.COLOR_BGR2GRAY)
    transform = cv.dilate(transform, kernel, iterations=10)
    transform = cv.bitwise_not(transform)

    counters, heir = cv.findContours(transform, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

    squares = []
    for cnt in counters:
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.04 * peri, True)

        if len(approx) == 4 and min_size < cv.contourArea(cnt) < max_size:
            M = cv.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            squares.append((cnt, (cx, cy)))

    if len(squares) == 0:
        if record:
            return [img], False
        return img, False

    nearest_square = squares[0]

    if test:
        print(cv.contourArea(nearest_square[0]))

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
            cv.circle(img, (a[0][0], a[0][1]), 15, (255, 0, 0), -1)

    pts1 = np.float32([approx[0][0], approx[1][0], approx[2][0], approx[3][0]])
    pts2 = np.float32([[0, 0], [0, 800], [800, 800], [800, 0]])

    M = cv.getPerspectiveTransform(pts1, pts2)

    perspective = cv.warpPerspective(img, M, (800, 800))

    if test:
        cv.imshow('perspective', perspective)
        resize2 = cv.resize(transform, (480, 480), interpolation=cv.INTER_AREA)
        cv.imshow('trans', resize2)
        k = cv.waitKey(1)
        if k == ord('q'):
            cv.destroyAllWindows()
    if record:
        return [img, perspective, transform, thresh], True
    return perspective, True


if __name__ == "__main__":
    # img = cv.imread(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\test_photo_4.png")
    # find_board(img, test=True)

    test = False
    cap = cv.VideoCapture(r"C:\Users\Arilon\Desktop\Projects\MarkerBot\1.mp4")

    ret, frame = cap.read()
    print(ret, (frame.shape[0:2]))
    w, h, sz = frame.shape
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    video_writer_1 = cv.VideoWriter("perspective2.avi", fourcc, 24.0, (800, 800))
    video_writer_2 = cv.VideoWriter("transform2.avi", fourcc, 24.0, (480, 480))
    video_writer_3 = cv.VideoWriter("thresh2.avi", fourcc, 24.0, (480, 480))
    video_writer_4 = cv.VideoWriter("img2.avi", fourcc, 24.0, (h, w))

    if test:
        cv.namedWindow("result")
        cv.namedWindow("settings")
        cv.createTrackbar('th_hough', 'settings', 0, 255, nothing)
        cv.createTrackbar('min_line', 'settings', 0, 4000, nothing)
        cv.createTrackbar('gap', 'settings', 0, 500, nothing)
        cv.createTrackbar('th_can_down', 'settings', 0, 255, nothing)
        cv.createTrackbar('th_can_up', 'settings', 0, 255, nothing)
        cv.createTrackbar('min_size', 'settings', 100000, 1000000, nothing)

    prev_images = []
    count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print('not')
            break
        images, flag = find_board(frame, test=False, record=True)
        if flag:
            images[2] = cv.resize(images[2], (480, 480), interpolation=cv.INTER_AREA)
            images[3] = cv.resize(images[3], (480, 480), interpolation=cv.INTER_AREA)
            images[2] = cv.cvtColor(images[2], cv.COLOR_GRAY2BGR)
            images[3] = cv.cvtColor(images[3], cv.COLOR_GRAY2BGR)
            video_writer_1.write(images[1])
            video_writer_2.write(images[2])
            video_writer_3.write(images[3])
            video_writer_4.write(images[0])
            #print(images[0].shape, images[2].shape, images[3].shape)#
            prev_images = images
        elif len(prev_images) > 1:
            video_writer_4.write(images[0])
            video_writer_1.write(prev_images[1])
            video_writer_2.write(prev_images[2])
            video_writer_3.write(prev_images[3])

    cap.release()
    video_writer_1.release()
    video_writer_2.release()
    video_writer_3.release()
    video_writer_4.release()
        # cv.imshow("res_img", img)
        # if cv.waitKey(1) == ord("q"):
        #     cap.release()
        #     exit()
