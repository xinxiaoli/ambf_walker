import rospy
import ambf_msgs.msg as ambf
import math
import numpy as np
from geometry_msgs.msg import Pose, Wrench
from Controller import PD_Controller
from std_msgs.msg import Float32MultiArray

from Model import Model
class BodyController():


    def __init__(self, model):
        """

        :type model: Model.Model
        """
        self.pub= rospy.Publisher('tau',Float32MultiArray, queue_size=1)
        self._model = model
        self.theta = np.linspace(0,-0.5,100)
        self.count = 0

    def calc_gravity(self):



        self.count = min(self.count, 99)
        Kp = np.zeros((6,6))
        Kd = np.zeros((6, 6))
        Kp[0,0] = 85
        Kd[0, 0] = 10.0

        qdd = np.zeros(6)
        g = self._model.calculate_dynamics(qdd)
        print self.count
        print self.theta
        tau = g + Kp.dot(np.array([self.theta[self.count], 0, 0, 0.0, 0, 0.0 ]) - self._model.q) + Kd.dot(np.array([-0.0, 0, 0, 0, 0, 0 ]) -  self._model.qd)

        msg = Float32MultiArray()
        msg.data = tau.tolist()
        self.pub.publish(msg)
        self._model.send_torque(tau)
        self.count += 1