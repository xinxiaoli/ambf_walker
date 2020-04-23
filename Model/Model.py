
import abc
import numpy as np
import rbdl
import time
import rospy
from threading import Thread
from lib.GaitCore.Bio import Joint, Leg
from lib.GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray

class Model(object):

    def __init__(self, client, mass, height):
        self._mass = mass
        self._height = height
        self._client = client
        self._q = np.array([])
        self._qd = np.array([])
        self.tau = np.array([])
        self._handle = None
        self._model = self.dynamic_model(mass, height)
        self._updater = Thread(target=self.update)
        self._enable_control = False

        left_joints = {}
        right_joints = {}

        for joint in (left_joints, right_joints):
            for output in ["Hip", "Knee", "Ankle"]:
                angle = Point.Point(0, 0, 0)
                force = Point.Point(0, 0, 0)
                moment = Point.Point(0, 0, 0)
                power = Point.Point(0, 0, 0)
                joint[output] = Joint.Joint(angle, moment, power, force)
                # joint[output] = core.Newton.Newton(angle, force, moment, power)

        self._left_leg = Leg.Leg(left_joints["Hip"], left_joints["Knee"], left_joints["Ankle"])
        self._right_leg = Leg.Leg(right_joints["Hip"], right_joints["Knee"], right_joints["Ankle"])


    def update_torque(self, tau):
        self.tau = tau
        self._enable_control = True

    @property
    def enable_control(self):
        return self._enable_control

    @enable_control.setter
    def enable_control(self, value):
        self._enable_control = value

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, value):
        self._handle = value

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, value):
        self._q = np.asarray(value)

    @property
    def qd(self):
        return self._qd

    @qd.setter
    def qd(self, value):
        # value[1] *= -1
        # value[9] *= -1
        self._qd = np.asarray(value)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    @abc.abstractmethod
    def dynamic_model(self, total_mass, height):
        pass

    def update(self):
        """

        :return:
        """
        rate = rospy.Rate(1000)  # 1000hz
        pub_qd = rospy.Publisher('qd', Float32MultiArray, queue_size=1)
        msg = Float32MultiArray()
        while 1:

            self.q = self.handle.get_all_joint_pos()
            self.qd = self.handle.get_all_joint_vel()
            self._joint_num = self.q.size
            if self._enable_control:
                # print(self.tau)

                self.handle.set_all_joint_effort(self.tau)
            pub_qd.publish(msg)
            rate.sleep()

    @abc.abstractmethod
    def ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        pass

    @abc.abstractmethod
    def update_state(self, q, qd):
        self.state = q + qd

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q[0:6], self.qd[0:6], qdd[0:6], tau)
        return tau

    def get_right_leg(self):
        """
        :return:
        """
        return self._right_leg

    def get_left_leg(self):
        """
        :return:
        """
        return self._left_leg