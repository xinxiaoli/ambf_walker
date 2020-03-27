
import abc
import numpy as np
import rbdl
import time
from threading import Thread


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
        while 1:
            self.q = self.handle.get_all_joint_pos()
            self.qd = np.zeros(self._model.qdot_size)
            qdd = np.zeros(self._model.qdot_size)
            rbdl.UpdateKinematics(self._model, self.q, self.qd, qdd)

    @abc.abstractmethod
    def  ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        pass

    @abc.abstractmethod
    def calculate_dynamics(self, q_d, qd_d, qdd_d):
        pass
