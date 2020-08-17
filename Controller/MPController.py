

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase
from GaitAnaylsisToolkit.LearningTools.Models import ModelBase

class MPController(ControllerBase.BaseController):

    def __init__(self, model, runner):
        """

        :param model:
        :param kp:
        :param kd:
        """
        self._runner = runner
        self.max_steps = 0
        super(MPController, self).__init__(model)

    def initilzie(self):
        count = 0
        self.u = []
        self.max_steps = self._runner.get_length()
        while count < self._runner.get_length():
            self._runner.step()
            self.u.append(self._runner.ddx)
            self.u.append(self._runner.x)

    def run_iLQR(self):

        A, b, J = self.foward_pass()
        while eps < 0.01:
            P, K = self.back_pass(A,b)
            A, b, J_ = self.foward_pass(_, K, P)
            eps = abs(J-J_)/J
            J = J_

    def foward_pass(self, x, K, P):
        count = 0
        h = 0.01
        A_ = []
        b_ = []
        expData = self.get_expData()
        v0 = np.zeros(len(x)).reshape((-1, 1))
        J = 0

        while count < self.max_steps:
            # add ut here
            u = K.dot(np.vstack((expData()[:, count].reshape((-1,1)), v0)) - x)
            y = Model.runge_integrator(self._model.model, y, h, u)
            x = y[:3]
            A, b = Model.finite_differences(y, u)
            A_.append(A)
            b_.append(b)
            J += np.dot(np.dot(y.reshape((-1, 1)).T, (P[count])), y.reshape((-1, 1)))
            count += 1

        return A, b, J


    def back_pass(self, A, b):
        # need to get cost
        expSigma = self.runner._data["expSigma"]
        ric = ModelBase.solve_riccati_mat(expSigma, A, b)
        P = ric["P"]
        K = ric["K"]
        return P, K

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


