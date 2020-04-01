import rospy
import ambf_msgs.msg as ambf
import math
import numpy as np
from geometry_msgs.msg import Pose, Wrench
from std_msgs.msg import Float32MultiArray

from Model import Model

class TestController():


    def __init__(self, model):
        """

        :type model: Model.Model
        """
        self.pub= rospy.Publisher('tau',Float32MultiArray, queue_size=1)
        self.pub_goal = rospy.Publisher('goal', Float32MultiArray, queue_size=1)
        self._model = model
        self.msg_goal = Float32MultiArray()
        goal = [-0.5, 0.0, 0.0]
        self.msg_goal.data = goal
        self.hip = np.linspace(0, goal[0], 25)
        self.knee = np.linspace(0, goal[1], 25)
        self.ankle = np.linspace(-0.349, goal[1], 25)

        self.count = 0

    def calc_gravity(self):

        self.count = min(self.count, 24)
        Kp = np.zeros((6,6))
        Kd = np.zeros((6, 6))
        Kp[0,0] = 85
        Kd[0, 0] = 10.0
        Kp[1, 1] = 0.0
        Kd[1, 1] = 0.0
        Kp[2, 2] = 15.00
        Kd[2, 2] = 0.10
        qdd = np.zeros(6)
        g = self._model.calculate_dynamics(qdd)
        tau = g + Kp.dot(np.array([self.hip[self.count], self.knee[self.count], self.ankle[self.count], 0.0, 0, 0.0]) - self._model.q) + Kd.dot(np.array([-0.0, 0, 0, 0, 0, 0]) - self._model.qd)
        msg = Float32MultiArray()
        msg.data = tau.tolist()
        self.pub.publish(msg)
        self.pub_goal.publish(self.msg_goal)
        self._model.send_torque(tau)
        self.count += 1