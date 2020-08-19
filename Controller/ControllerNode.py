import rospy
from threading import Thread
import threading
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
import numpy as np
from std_msgs.msg import Float32MultiArray
from . import DynController
from ambf_walker.srv import DesiredJointsCmd, DesiredJointsCmdResponse


class ControllerNode(object):

    def __init__(self, model, controllers):

        self._model = model
        self._controllers = controllers
        self.controller = self._controllers["Dyn"]
        self._updater = Thread(target=self.set_torque)
        self.lock = threading.Lock()
        self.sub_set_points = rospy.Subscriber("set_points", DesiredJoints, self.update_set_point)
        self.tau_pub = rospy.Publisher("joint_torque", JointState, queue_size=1)
        self.traj_pub = rospy.Publisher("trajectory", Float32MultiArray, queue_size=1)
        self.error_pub = rospy.Publisher("Error", Float32MultiArray, queue_size=1)
        self.service = rospy.Service('joint_cmd', DesiredJointsCmd, self.joint_cmd_server)
        self._enable_control = False
        self.ctrl_list = []
        self.q = np.array([])
        self.qd = np.array([])
        self.qdd = np.array([])

    def update_set_point(self, msg):
        """

        :type msg: DesiredJoints
        """
        self.lock.acquire()
        print("potato " + msg.controller)
        self.controller = self._controllers[msg.controller]
        self.q = np.array(msg.q)
        self.qd = np.array(msg.qd)
        self.qdd = np.array(msg.qdd)
        self.lock.release()
        if not self._enable_control:
            self._updater.start()
        return True

    def joint_cmd_server(self, msg):
        joints = DesiredJoints()
        joints.q = msg.q
        joints.qd = msg.qd
        joints.qdd = msg.qdd
        joints.controller = msg.controller
        print(msg.controller)
        good = self.update_set_point(joints)
        return DesiredJointsCmdResponse(good)

    def set_torque(self):
        self._enable_control = True
        rate = rospy.Rate(1000)
        tau_msg = JointState()
        traj_msg = Float32MultiArray()
        error_msg = Float32MultiArray()

        while 1:
            tau = self.controller.calc_tau(self.q, self.qd, self.qdd)
            #error_msg.data = abs(self.q - self._model.q)
            tau_msg.effort = tau.tolist()
            #traj_msg.data = self.q
            self.tau_pub.publish(tau_msg)
            #self.traj_pub.publish(traj_msg)
            #self.error_pub.publish(error_msg)
            rate.sleep()

