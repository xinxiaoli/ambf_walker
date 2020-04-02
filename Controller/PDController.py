

import numpy as np


class PDController(object):

    def __init__(self, kp, kd):

        self._kp = kp
        self._kd = kd


    @property
    def kp(self):
        return self._kp

    @kp.setter
    def kp(self, kp):
        self._kp = kp

    @property
    def kd(self):
        return self._kd

    @kd.setter
    def kd(self, kd):
        self._kd = kd

    def get_tau(self, e, ed):
        return self.kp.dot(e) + self.kd.dot(ed)