import matplotlib.pyplot as plt
from threading import Thread
import time
from mpl_toolkits.mplot3d import Axes3D

plt.ion()



class ThreeDPlotter():

    def __init__(self, model):
        self.model = model
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, projection='3d' )
        self.ax.set_zlim3d( -1.25, 0.25 )
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.sc = self.ax.scatter([], [], [])
        #self.fig.show()

    def update(self):
        points = self.model.fk()

        z = [0, points["left_hip"].z, points["left_knee"].z, points["left_ankle"].z, points["left_toe"].z,
            points["left_heel"].z, points["left_ankle"].z]
        y = [0, points["left_hip"].y, points["left_knee"].y, points["left_ankle"].y, points["left_toe"].y,
            points["left_heel"].y, points["left_ankle"].y]

        x = [0, points["left_hip"].x, points["left_knee"].x, points["left_ankle"].x, points["left_toe"].x,
            points["left_heel"].x, points["left_ankle"].x]

        print "left ", points["left_hip"]

        print "Right ", points["right_hip"]


        z += [0, points["right_hip"].z, points["right_knee"].z, points["right_ankle"].z, points["right_toe"].z,
             points["right_heel"].z, points["right_ankle"].z]
        y += [0, points["right_hip"].y, points["right_knee"].y, points["right_ankle"].y, points["right_toe"].y,
             points["right_heel"].y, points["right_ankle"].y]

        x += [0, points["right_hip"].x, points["right_knee"].x, points["right_ankle"].x, points["right_toe"].x,
             points["right_heel"].x, points["right_ankle"].x]
        self.sc._offsets3d = (x, y, z)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class Plotter(object):

    def __init__(self, model):
        self.model = model
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-0.6, 0.2), ylim=(-1.2, 0.3))
        self.ax.grid()
        self.right_leg, = self.ax.plot([0,0,0], [0,-0.5,-0.5], 'ro-', lw=2)
        self.left_leg, = self.ax.plot([0,0,0], [0,-0.5,-0.5], 'bo-', lw=2)
        self.trunk, = self.ax.plot([], [], 'go-', lw=2)
        # self.__updater = Thread(target=self.update())
        # self.__updater.start()


    def update(self):

        points = self.model.fk()

        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')

        self.trunk.set_ydata([0, 0.25])
        self.trunk.set_xdata([0, 0])



        self.right_leg.set_ydata(
            [0, points["right_hip"].z, points["right_knee"].z, points["right_ankle"].z, points["right_toe"].z])
        self.right_leg.set_xdata(
            [0, points["right_hip"].y, points["right_knee"].y, points["right_ankle"].y, points["right_toe"].y])

        self.left_leg.set_ydata(
            [0, points["left_hip"].z, points["left_knee"].z, points["left_ankle"].z, points["left_toe"].z])
        self.left_leg.set_xdata(
            [0, points["left_hip"].y, points["left_knee"].y, points["left_ankle"].y, points["left_toe"].y])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

