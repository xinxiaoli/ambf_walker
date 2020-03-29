import matplotlib.pyplot as plt
from threading import Thread
import time
from mpl_toolkits.mplot3d import Axes3D

plt.ion()


class Plotter(object):

    def __init__(self, model):
        self.model = model
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-0.7, 0.7), ylim=(-1.25, 0.25))
        self.ax.grid()
        self.left_leg, = self.ax.plot([0,0,0], [0,-0.5,-0.5], 'bo-', lw=2)
        self.right_leg, = self.ax.plot([0,0,0], [0,-0.5,-0.5], 'ro-', lw=2)
        self.trunk, = self.ax.plot([], [], 'go-', lw=2)
        # self.__updater = Thread(target=self.update())
        # self.__updater.start()


    def update(self):

        points = self.model.fk()

        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')

        self.trunk.set_ydata([0, 0.25])
        self.trunk.set_xdata([0, 0])

        self.left_leg.set_ydata(
            [0, points["left_hip"].z, points["left_knee"].z, points["left_ankle"].z, points["left_toe"].z,
             points["left_heel"].z, points["left_ankle"].z])
        self.left_leg.set_xdata(
            [0, points["left_hip"].x, points["left_knee"].x, points["left_ankle"].x, points["left_toe"].x,
             points["left_heel"].x, points["left_ankle"].x])

        self.right_leg.set_ydata(
            [0, points["right_hip"].z, points["right_knee"].z, points["right_ankle"].z, points["right_toe"].z,
             points["right_heel"].z, points["right_ankle"].z])
        self.right_leg.set_xdata(
            [0, points["right_hip"].x, points["right_knee"].x, points["right_ankle"].x, points["right_toe"].x,
             points["right_heel"].x, points["right_ankle"].x])


        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

