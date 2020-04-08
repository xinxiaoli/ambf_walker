
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
        self._handle = None
        self._model = self.dynamic_model(mass, height)
        self._updater = Thread(target=self.update)

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

    def send_torque(self, tau):
        self.handle.set_all_joint_effort(tau)

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
        self._q = np.asarray(value) #  -np.array([-0.4, -0.157, 0.349, 0.0,0,0 ])

    @property
    def qd(self):
        return self._qd

    @qd.setter
    def qd(self, value):
        value[2] *= -1
        value[5] *= -1
        self._qd = np.asarray(value[0:6])

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

            q = self.handle.get_all_joint_pos()
            self.q = q
            self.qd = self.handle.get_all_joint_vel()
            self._joint_num = len(q)
            self.update_state(self.q, self.qd)
            pub_qd.publish(msg)
            rate.sleep()

    @abc.abstractmethod
    def  ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        pass

    @abc.abstractmethod
    def update_state(self, q, qd):
        self.state = q + qd

    def calculate_dynamics(self, qdd):
        # self.q = self.handle.get_all_joint_pos()
        # self.qd = self.handle.get_all_joint_vel()
        # print self.q
        tau = np.asarray([0.0] * self._joint_num)
        print "tau ", self._joint_num
        #self.qd = np.array(6*[0.0])
        rbdl.InverseDynamics(self._model, self.q, self.qd, qdd, tau)
        return tau
        #return np.array([-0.47484118,  0.42351732,  0.50338838, -0.47484118,  0.42351732,  0.50338838])

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