

import numpy as np
from . import ControllerBase

class PDController(ControllerBase.BaseController):

    def __init__(self, model, kp, kd):
        super(PDController, self).__init__(model)
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

    def calc_tau(self, q=None, qd=None, qdd=None):
        return self.kp.dot(q) + self.kd.dot(qd)