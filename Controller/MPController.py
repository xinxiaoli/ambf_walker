

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase

class MPController(ControllerBase.BaseController):

    def __init__(self, model, runner):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._runner = runner
        super(MPController, self).__init__(model)

    def foward_pass(self, y):
        count = 0
        h = 0.01
        A_ = []
        b_ = []
        while count < self._runner.get_length():
            self._runner.step()
            u = self._runner.ddx
            y = Model.runge_integrator(self._model.model, y, h, u)
            A, b = Model.finite_differences(y, u)
            A_.append(A)
            b_.append(b)
            count += 1


    def back_pass(self, A, b):

        expSigma = self.runner._data["expSigma"]
        size = expSigma[0].shape[0]
        R = np.eye(size) * 0.001 # reg[1:]
        Q = np.zeros((size * 2, size * 2))
        P = [np.zeros((size * 2, size * 2))] * len(expSigma)
        P[-1][:size, :size] = np.linalg.pinv(expSigma[-1])

        for ii in range(len(expSigma) - 2, -1, -1):
            Q[:size, :size] = np.linalg.pinv(expSigma[ii])
            B = P[ii + 1].dot(b)
            C = np.linalg.pinv(np.dot(b.T.dot(P[ii + 1]), b) + R)
            D = b.T.dot(P[ii + 1])
            F = np.dot(np.dot(A.T, B.dot(C).dot(D) - P[ii + 1]), A)
            P[ii] = Q - F
            

    def calc_tau(self, q=None, qd=None, qdd=None):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        aq = np.zeros(len(q))
        if q is not None and qd is not None:
            e = q - self._model.q
            ed = qd - self._model.qd
            aq = self.pdController.calc_tau(e, ed)
            aq += qdd
        tau = self._model.calculate_dynamics(aq)
        return tau


