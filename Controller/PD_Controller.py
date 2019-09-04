import numpy as np

class PDController():

    def __init__(self, Kp, Kd):

        self._Kp = Kp
        self._Kd = Kd

    def calc(self, e, ed):
        force= np.multiply(self.Kp, e) # + np.multiply(self.Kd, ed)
        return force

    @property
    def Kp(self):
        return self._Kp

    @Kp.setter
    def Kp(self, value):
        self._Kp = value

    @property
    def Kd(self):
        return self._Kd

    @Kd.setter
    def Kd(self, value):
        self._Kd = value
