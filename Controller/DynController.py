import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
import PDController


class DynController(object):

    def __init__(self, model, kp, kd):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._model = model
        self.pdController = PDController.PDController(kp, kd)
        self.pub = rospy.Publisher('tau', Float32MultiArray, queue_size=1)

    def set_gains(self, kp, kd):
        """

        :param kp:
        :param kd:
        :return:
        """
        self.pdController.kp = kp
        self.pdController.kd = kd

    def calc_tau(self, q=None, qd=None, qdd=None, controllers=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        aq = np.zeros(7)
        if q is not None and qd is not None:
            e = q - self._model.q
            ed = qd - self._model.qd
            aq = self.pdController.get_tau(e, ed)

        if qdd is not None:
            aq += qdd

        for ii, ctrl in enumerate(controllers):
            if ctrl is "LQR":
                aq[ii] = qdd[ii]

        return aq


class DynControllerNode(object):

    def __init__(self, model, kp, kd):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._model = model
        self.pdController = PDController.PDController(kp, kd)
        self.pub = rospy.Publisher('tau', Float32MultiArray, queue_size=1)

    def set_gains(self, kp, kd):
        """

        :param kp:
        :param kd:
        :return:
        """
        self.pdController.kp = kp
        self.pdController.kd = kd

    def calc_tau(self, q=None, qd=None, qdd=None, controllers=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        aq = np.zeros(7)
        tau = np.zeros(7)
        aq[0] = qdd[0]
        # if q is not None and qd is not None:
        #     e = q - self._model.q
        #     ed = qd - self._model.qd
        #     aq = self.pdController.get_tau(e, ed)
        #
        # if qdd is not None:
        #     aq = qdd + aq
        #
        # for ii, cnrl in enumerate(controllers):
        #     if cnrl == "LQR":
        #         aq[ii] = qdd[ii]

        temp = self._model.calculate_dynamics(aq)
        #tau[0] = temp[0]
        msg = Float32MultiArray()
        msg.data = tau.tolist()
        self.pub.publish(msg)
        self._model.update_torque(tau)