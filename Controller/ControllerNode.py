import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
import numpy as np
from std_msgs.msg import Float32MultiArray
from . import DynController
class ControllerNode(object):

    def __init__(self, model):

        self._model = model
        # rospy.init_node('Controller')
        self._updater = Thread(target=self.set_torque)
        self.sub_set_points = rospy.Subscriber("set_points", DesiredJoints, self.update_set_point)
        self.tau = rospy.Publisher("joint_torque", JointState, queue_size=1)
        self.traj = rospy.Publisher("trajectory", Float32MultiArray, queue_size=1)
        self._enable_control = False
        self.ctrl_list = []
        self.q = np.array([])
        self.qd = np.array([])
        self.qdd = np.array([])

        Kp = np.zeros((7, 7))
        Kd = np.zeros((7, 7))

        Kp_hip = 50.0
        Kd_hip = 0.5

        Kp_knee = 125.0
        Kd_knee = 1.0

        Kp_ankle = 100.0
        Kd_ankle = 0.4

        Kp[0, 0] = Kp_hip
        Kd[0, 0] = Kd_hip
        Kp[1, 1] = Kp_knee
        Kd[1, 1] = Kd_knee
        Kp[2, 2] = Kp_ankle
        Kd[2, 2] = Kd_ankle

        Kp[3, 3] = Kp_hip
        Kd[3, 3] = Kd_hip
        Kp[4, 4] = Kp_knee
        Kd[4, 4] = Kd_knee
        Kp[5, 5] = Kp_ankle
        Kd[5, 5] = Kd_ankle
        self.controller = DynController.DynController(model, Kp, Kd)

    def update_set_point(self, msg):
        """

        :type msg: DesiredJoints
        """

        self.q = np.array(msg.q)
        self.qd = np.array(msg.qd)
        self.qdd = np.array(msg.qdd)
        self.ctrl_list = msg.controllers
        if not self._enable_control:
            self._updater.start()

    def set_torque(self):
        self._enable_control = True
        rate = rospy.Rate(1000)
        tau_msg = JointState()
        traj_msg = Float32MultiArray()
        while 1:
            aq = self.controller.calc_tau(self.q, self.qd, self.qdd, self.ctrl_list)
            tau = self._model.calculate_dynamics(aq)
            tau_msg.effort = tau.tolist()
            traj_msg.data = self.q
            self.tau.publish(tau_msg)
            self.traj.publish(traj_msg)
            rate.sleep()

