import rospy
import abc
from threading import Thread
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Float32MultiArray

class BaseController(object):

    def __init__(self, model):

        self._model = model
        # rospy.init_node('Controller')
        self._updater = Thread(target=self.set_torque)
        self.tau = rospy.Publisher("joint_torque", JointState, queue_size=1)
        self.traj = rospy.Publisher("trajectory", Float32MultiArray, queue_size=1)
        self._enable_control = False
        self.ctrl_list = []
        self.q = np.array([])
        self.qd = np.array([])
        self.qdd = np.array([])


    def update_set_point(self, msg):
        """

        :type msg: DesiredJoints
        """

        self.q = np.array(msg.position)
        self.qd = np.array(msg.velocity)
        self.qdd = np.array(msg.effort)
        if not self._enable_control:
            self._updater.start()

    @abc.abstractmethod
    def calc_tau(self, q, qd, qdd):
        pass

    def set_torque(self):
        self._enable_control = True
        rate = rospy.Rate(1000)
        tau_msg = JointState()
        traj_msg = Float32MultiArray()
        while 1:
            tau_msg.effort = self.calc_tau(self.q, self.qd, self.qdd)
            traj_msg.data = self.q
            self.tau.publish(tau_msg)
            self.traj.publish(traj_msg)
            rate.sleep()

