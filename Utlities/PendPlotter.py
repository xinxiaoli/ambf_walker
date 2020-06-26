import matplotlib.pyplot as plt

import rbdl

from Models import double_pendulm

plt.ion()


# TODO create overload updates to plot with/without RBDL
class Plotter(object):

    def __init__(self, model):
        self.model = model
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, autoscale_on=False, xlim=(-1, 1), ylim=(-2,2))
        self.ax.grid()
        self.stem, = self.ax.plot([], [], 'bo-', lw=2)


    def update(self):

        x, y = self.model.fk()
        self.stem.set_xdata(x)
        self.stem.set_ydata(y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()