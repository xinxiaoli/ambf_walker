import matplotlib.pyplot as plt
from threading import Thread
import time
from mpl_toolkits.mplot3d import Axes3D

plt.ion()


class Plotter(object):

    def __init__(self, model):
        self.model = model
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-1.0, 1.0), ylim=(-1.0, 1.0))
        self.ax.grid()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_autoscale_on(False)
        self.left_leg, = self.ax.plot([], [], 'bo-', lw=2)
        self.right_leg, = self.ax.plot([], [], 'ro-', lw=2)
        # self.trunk, = self.ax.plot([], [], 'go', lw=2)
        # self.__updater = Thread(target=self.update())
        # self.__updater.start()


    def update(self):

        points = self.model.fk()
        print points
        y = []
        x = []
        z = []

        # for p in points:
        #     print p
        #     x.append(p[0])
        #     y.append(p[1])
        #     z.append(p[2])
        # print "--------------------------------------------------"
        self.ax.clear()
        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')


        # self.trunk.set_xdata(x)
        # self.trunk.set_ydata(y)

        # self.left_leg.set_ydata(
        #     [points["left_hip"]["y"], points["left_knee"]["y"], points["left_ankle"]["y"], points["left_toe"]["y"],
        #      points["left_heel"]["y"], points["left_ankle"]["y"]])
        #
        # self.left_leg.set_xdata(
        #     [points["left_hip"]["x"], points["left_knee"]["x"], points["left_ankle"]["x"], points["left_toe"]["x"],
        #      points["left_heel"]["x"], points["left_ankle"]["x"]])
        # # for key, item in points.iteritems():
        # #     print key + ":"
        # #     print "x: " + str(points[key]["x"]) + " y: " + str(points[key]["y"]) + " z: " + str(points[key]["z"])
        # # print "-----------------------------------------------------------------------"
        # self.right_leg.set_ydata(
        #     [points["right_hip"]["y"], points["right_knee"]["y"], points["right_ankle"]["y"], points["right_toe"]["y"],
        #      points["right_heel"]["y"], points["right_ankle"]["y"]])
        # self.right_leg.set_xdata(
        #     [points["right_hip"]["x"], points["right_knee"]["x"], points["right_ankle"]["x"], points["right_toe"]["x"],
        #      points["right_heel"]["x"], points["right_ankle"]["x"]])
        #
        # # self.left_leg.set_ydata([legs_y[5], legs_y[6], legs_y[7], R[0][1], R[1][1], legs_y[7]])
        # # self.left_leg.set_xdata([legs_x[5], legs_x[6], legs_x[7], R[0][0], R[1][0], legs_x[7]])
        # #
        # # self.right_leg.set_ydata([legs_y[2], legs_y[3], legs_y[4], L[0][1], L[1][1], legs_y[4]])
        # # self.right_leg.set_xdata([legs_x[2], legs_x[3], legs_x[4], L[0][0], L[1][0], legs_x[4]])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

