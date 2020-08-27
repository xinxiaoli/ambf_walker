import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from Model import Model
from . import PDController
from sensor_msgs.msg import JointState
from . import ControllerBase
from ilqr import iLQR, RecedingHorizonController
from ilqr.cost import QRCost, PathQRCost, PathQsRCost
from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
from ilqr.dynamics import AutoDiffDynamics, BatchAutoDiffDynamics, FiniteDiffDynamics
import matplotlib.pyplot as plt

class LQRController(ControllerBase.BaseController):

    def __init__(self, model, runner):
        """

        :param model:
        :param kp:
        :param kd:
        """
        super(LQRController, self).__init__(model)
        self.runner = runner
        self.x = []
        self.setup()

    def setup(self):

        J_hist = []

        def on_iteration(iteration_count, xs, us, J_opt, accepted, converged):
            J_hist.append(J_opt)
            info = "converged" if converged else ("accepted" if accepted else "failed")
            print("iteration", iteration_count, info, J_opt)

        def f(x, u, i):
            y = Model.runge_integrator(self._model.get_rbdl_model(), x, 0.01, u)

            return np.array(y)

        dynamics = FiniteDiffDynamics(f, 12, 6)


        x_path = []
        u_path = []
        count = 0
        N = self.runner.get_length()
        while count < self.runner.get_length():
            count += 1
            self.runner.step()
            u_path.append(self.runner.ddx.flatten().tolist())
            self.x.append(self.runner.x.flatten())
            x = self.runner.x.flatten().tolist() + self.runner.dx.flatten().tolist()
            x_path.append(x)

        u_path = u_path[:-1]
        expSigma = self.runner.get_expSigma()
        size = expSigma[0].shape[0]
        Q = [np.zeros((size * 2, size * 2))] * len(expSigma)
        for ii in range(len(expSigma) - 2, -1, -1):
            Q[ii][:size, :size] = np.linalg.pinv(expSigma[ii])

        x0 = x_path[0]
        x_path = np.array(x_path)
        u_path = np.array(u_path)
        R = 0.00005 * np.eye(dynamics.action_size)
        #
        cost2 = PathQsRCost(Q, R, x_path=x_path, u_path=u_path)
        #
        # # Random initial action path.
        us_init = np.random.uniform(-1, 1, (N-1, dynamics.action_size))
        #
        J_hist = []
        ilqr = iLQR(dynamics, cost2, N-1)
        self.xs, self.us = ilqr.fit(x0, us_init, on_iteration=on_iteration)

        # max_bounds = 0.1
        # min_bounds = -0.1
        # diff = (max_bounds - min_bounds) / 2.0
        # mean = (max_bounds + min_bounds) / 2.0
        # self.us[:, 0] = diff * np.tanh(self.us[:, 0]) + mean
        # self.us[:, 3] = diff * np.tanh(self.us[:, 3]) + mean
        #
        # max_bounds = 1.0
        # min_bounds = -1.0
        # diff = (max_bounds - min_bounds) / 2.0
        # mean = (max_bounds + min_bounds) / 2.0
        # self.us[:, 1] = diff * np.tanh(self.us[:, 1]) + mean
        # self.us[:, 4] = diff * np.tanh(self.us[:, 4]) + mean
        #
        # max_bounds = 0.77
        # min_bounds = -0.77
        # diff = (max_bounds - min_bounds) / 2.0
        # mean = (max_bounds + min_bounds) / 2.0
        # self.us[:, 2] = diff * np.tanh(self.us[:, 2]) + mean
        # self.us[:, 5] = diff * np.tanh(self.us[:, 5]) + mean

    def calc_tau(self, q=None, qd=None, qdd=None, other=None ):
        """

        :param q:
        :param qd:
        :param qdd:
        :return:
        """
        #print(self.us[int(other[0])])

        #tau = self._model.calculate_dynamics( np.append(self.us[int(other[0])], [0.0]) )
        tau = self._model.grav( self.x[(int(other[0]))]) + np.append(self.us[int(other[0])], [0.0])
        return tau