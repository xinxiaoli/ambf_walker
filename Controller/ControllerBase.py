import rospy
import numpy as np
import abc
from sensor_msgs.msg import JointState

class BaseController(object):

    def __init__(self, model):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._model = model
        self.pub = rospy.Publisher('tau', JointState, queue_size=1)

    @abc.abstractmethod
    def calc_tau(self, q=None, qd=None, qdd=None, other=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        raise NotImplemented

