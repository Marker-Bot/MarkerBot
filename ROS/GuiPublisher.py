#!/usr/bin/env python
import rospy
from hrwros_msgs.msg import Points_arrays

from Tkinter import *
import numpy as np
import pandas as pd

# Node to publish a string topic.
def PointsPublisher():
	p_publisher = rospy.Publisher('points_from_gui_topic', Points_arrays, queue_size = 10)
	rospy.init_node('gui_publisher_node', anonymous = False)
	rate = rospy.Rate(10)

	# The array to be published on the topic.
	points_info = Points_arrays()
	points_info.x_coordinates = str(traj_x)
	points_info.y_coordinates = str(traj_y)
	
	#while not rospy.is_shutdown():
        p_publisher.publish(points_info)
        #	rate.sleep()


traj_x = []
traj_y = []

def clear_traj():
    del traj_x[:]
    del traj_y[:]


class Paint(object):
    default_brush_size = 3
    default_colour = 'black'

    def __init__(self):
        board_width = 800
        board_height = 600
        self.root = Tk()
        self.root.title("A board")

        self.brush_button = Button(self.root, text='brush', command=self.use_brush)
        self.brush_button.grid(row=0, column=1)

        self.eraser_button = Button(self.root, text='eraser', command=self.use_eraser)
        self.eraser_button.grid(row=0, column=2)

        self.c = Canvas(self.root, bg='white', width=board_width, height=board_height)
        self.c.grid(row=1, columnspan=5)

        self.clear_btn = Button(self.root, text="clean the board", width=20, command=self.clean_board)
        self.clear_btn.grid(row=0, column=3, sticky=W)


        self.draw_it_button = Button(self.root, text='draw it!', command=self.sent_trajectory)
        self.draw_it_button.grid(row=0, column=4)

        self.setup()
        self.root.mainloop()

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = self.default_brush_size
        self.color = self.default_colour
        self.eraser_on = False
        self.active_button = self.brush_button
        self.c.bind('<B1-Motion>', self.paint)
        self.c.bind('<ButtonRelease-1>', self.reset)

    def clean_board(self):
        self.c.delete("all")
        clear_traj()
        self.use_brush()

    def sent_trajectory(self):
        PointsPublisher()

    def use_brush(self):
        self.activate_button(self.brush_button)

    def use_eraser(self):
        self.activate_button(self.eraser_button, eraser_mode=True)

    def activate_button(self, some_button, eraser_mode=False):
        self.active_button.config(relief=RAISED)
        some_button.config(relief=SUNKEN)
        self.active_button = some_button
        self.eraser_on = eraser_mode

    def paint(self, event):
        self.line_width = self.default_brush_size
        paint_color = 'white' if self.eraser_on else self.color
        if self.old_x and self.old_y:
            self.c.create_line(self.old_x, self.old_y, event.x, event.y,
                               width=self.line_width, fill=paint_color,
                               capstyle=ROUND, smooth=TRUE, splinesteps=36)
        self.old_x = event.x
        self.old_y = event.y
        traj_x.append(event.x)
        traj_y.append(-event.y+600)

    def reset(self, event):
        self.old_x, self.old_y = None, None


if __name__ == '__main__':
    Paint()


print(traj_x)
print(traj_y)
