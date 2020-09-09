import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase

class TempController(ControllerBase.BaseController):

    def __init__(self, model):
        """

        :param model:
        :param kp:
        :param kd:
        """
        super(TempController, self).__init__(model)

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

        tau = qdd
        return tau


