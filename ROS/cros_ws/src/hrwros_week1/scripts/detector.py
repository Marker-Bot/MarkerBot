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
	"""
	Нода детекции робота. Передает в string топик значения координат робота.

	"""
	p_publisher = rospy.Publisher('points_from_detector_topic', Points_arrays, queue_size = 10)
	rospy.init_node('detector_node', anonymous = False)
	rate = rospy.Rate(10)


	#Test arrays
	traj_x = []
	traj_y = []

	cap = cv.VideoCapture(1)
	previous_values = [None, None, None, None]

	while True:
		# Считывание изображения с камеры
		ret, frame = cap.read()
		if not ret:
			print('not')
			break
		center, res, angle = rd.detect_robot_coords(frame, previous_values)
		previous_values = [center, res, angle]

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
