#!/usr/bin/env python
# Node to subscribe to a string and print the string on terminal.
import rospy
import socket
from hrwros_msgs.msg import Points_arrays

import camera_vision.Client as cl

# Topic callback function.
def stringListenerCallback(data):
    cl.sendData(data.x_coordinates, data.y_coordinates)
    rospy.loginfo('X coordinates: %s', data.x_coordinates)
    rospy.loginfo('Y coordinates: %s', data.y_coordinates)

def stringListener():

    rospy.init_node('coordinates_server_node', anonymous = False)

    rospy.Subscriber('points_from_gui_topic', Points_arrays, stringListenerCallback)
    rospy.Subscriber('points_from_detector_topic', Points_arrays, stringListenerCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    stringListener()
