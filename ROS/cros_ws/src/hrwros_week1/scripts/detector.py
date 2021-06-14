#!/usr/bin/env python
# Node to subscribe to a string and print the string on terminal.
import rospy
from hrwros_msgs.msg import Points_arrays

# Node to publish a string topic.
def Detector():
	p_publisher = rospy.Publisher('points_from_detector_topic', Points_arrays, queue_size = 10)
	rospy.init_node('detector_node', anonymous = False)
	rate = rospy.Rate(10)

	#Test arrays
	traj_x = [1, 1, 1]
	traj_y = [1, 1, 1]
	
	# The array to be published on the topic.
	points_info = Points_arrays()
	points_info.x_coordinates = str(traj_x)
	points_info.y_coordinates = str(traj_y)
	
	#while not rospy.is_shutdown():
        p_publisher.publish(points_info)
        #rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
	Detector()
