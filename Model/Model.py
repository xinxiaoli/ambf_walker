
import abc
import numpy as np
import rbdl
import time
from threading import Thread
from lib.GaitCore.Bio import Joint, Leg
from lib.GaitCore.Core import Point

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
        self._q = -np.asarray(value)

    @property
    def qd(self):
        return self._qd

    @qd.setter
    def qd(self, value):
        self._qd = -np.asarray(value)

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
        while 1:
            q = self.handle.get_all_joint_pos()
            #q = np.zeros(self._model.qdot_size)
            qd = self.handle.get_all_joint_vel()
            qdd = np.zeros(self._model.qdot_size)
            self.update_state(q, qd)
            self.q = q
            self.qd = qd
            rbdl.UpdateKinematics(self._model, self.q, self.qd, qdd)

    @abc.abstractmethod
    def  ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        pass

    @abc.abstractmethod
    def update_state(self, q,qd):
        self.state = q + qd

    @abc.abstractmethod
    def calculate_dynamics(self, q_d, qd_d, qdd_d):
        pass

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