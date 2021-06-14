from tkinter import *
from tkinter.colorchooser import askcolor
import plotly
import plotly.graph_objs as go
import plotly.express as px
from plotly.subplots import make_subplots

import numpy as np
import pandas as pd

traj_x = []
traj_y = []


def clear_traj():
    traj_x.clear()
    traj_y.clear()


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

        # Команда, которая будет посылать рисунок
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

        print(traj_x)
        print(traj_y)

        layout = go.Layout(yaxis=dict(range=[0, 600]), xaxis=dict(range=[0, 800]))
        fig = go.Figure(layout=layout)
        fig.add_trace(go.Scatter(x=traj_x, y=traj_y, mode='markers'))
        fig.show()

        x_av = []
        y_av = []
        s = []
        r = []
        for i in range(3, len(traj_x)-1):
            if (abs(traj_x[i - 2] - traj_x[i]) < 3) and (abs(traj_y[i - 2] - traj_y[i]) < 3):
                s.append(traj_x[i - 2])
                r.append(traj_y[i - 2])

        l = []
        for i in range(len(s)):
            coords = tuple([s[i], r[i]])
            l.append(coords)

        q = []
        for j in range(len(traj_x)):
            coords1 = tuple([traj_x[j], traj_y[j]])
            q.append(coords1)

        for k in range(len(l)):
            q.remove(l[k])

        for g in range(len(q)):
            x_av.append((q[g])[0])
            y_av.append((q[g])[1])

        layout = go.Layout(yaxis=dict(range=[0, 600]), xaxis=dict(range=[0, 800]))
        fig = go.Figure(layout=layout)
        fig.add_trace(go.Scatter(x=x_av, y=y_av, mode='markers'))
        fig.show()



        self.c.delete("all")
        clear_traj()

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
        # Добавление координат в массив и преобразование в другую систему координат
        traj_x.append(event.x)
        traj_y.append(-event.y + 600)

    def reset(self, event):
        self.old_x, self.old_y = None, None


if __name__ == '__main__':
    Paint()