#!/usr/bin/env python
# Node to subscribe to a string and print the string on terminal.
import rospy
from hrwros_msgs.msg import Points_arrays
import cv2 as cv
import numpy as np


import camera_vision.board_detection as bd
import camera_vision.robot_detection as rd
import camera_vision.detector as dt


# Node to publish a string topic.
def Detector():
	p_publisher = rospy.Publisher('points_from_detector_topic', Points_arrays, queue_size = 10)
	rospy.init_node('detector_node', anonymous = False)
	rate = rospy.Rate(10)


	#Test arrays
	traj_x = []
	traj_y = []

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

		coord_x, coord_y = center
		traj_x.append(coord_x)
		traj_y.append(coord_y)


		# The array to be published on the topic.
		points_info = Points_arrays()
		points_info.x_coordinates = str(traj_x)
		points_info.y_coordinates = str(traj_y)

	
		#while not rospy.is_shutdown():
        	p_publisher.publish(points_info)
        	#rate.sleep()

		traj_x.clear()
		traj_y.clear()
		# spin() simply keeps python from exiting until this node is stopped
    		rospy.spin()

if __name__ == '__main__':
	Detector()
