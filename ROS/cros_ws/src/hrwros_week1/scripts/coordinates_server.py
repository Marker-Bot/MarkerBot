#!/usr/bin/env python
# Node to subscribe to a string and print the string on terminal.
import rospy
import socket
from hrwros_msgs.msg import Points_arrays

# Topic callback function.
def stringListenerCallback(data):
    rospy.loginfo('X coordinates: %s', data.x_coordinates)
    rospy.loginfo('Y coordinates: %s', data.y_coordinates)

def stringListener():
    #sock = socket.socket()
    #sock.connect(('ev3dev.local', 9090))
    #sock.send(x_coordinates.encode())
    #sock.send(y_coordinates.encode())

    #ev3_data = sock.recv(1024).decode()
    #sock.close()


    rospy.init_node('coordinates_server_node', anonymous = False)

    rospy.Subscriber('points_from_gui_topic', Points_arrays, stringListenerCallback)
    rospy.Subscriber('points_from_detector_topic', Points_arrays, stringListenerCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    stringListener()
