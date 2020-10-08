import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase

class DynController(ControllerBase.BaseController):

    def __init__(self, model, kp, kd):
        """

        :param model:
        :param kp:
        :param kd:
        """
        super(DynController, self).__init__(model)
        self.pdController = PDController.PDController(model, kp, kd)

    def set_gains(self, kp, kd):
        """

        :param kp:
        :param kd:
        :return:
        """
        self.pdController.kp = kp
        self.pdController.kd = kd

    def calc_tau(self, q=None, qd=None, qdd=None, other=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        aq = np.zeros(len(q))
        if q is not None and qd is not None:
            e = q - self._model.q
            ed = qd #- self._model.qd
            aq = self.pdController.calc_tau(e, ed)
            aq += qdd
        tau = self._model.calculate_dynamics(aq)
        return tau


