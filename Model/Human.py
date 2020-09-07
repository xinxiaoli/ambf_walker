"""
This should be moved to a seperate repo later
"""


import abc
import numpy as np
import rbdl
import Model
import time
from GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from threading import Thread


class Human(Model.Model):

    def __init__(self, client, mass, height):
        super(Human, self).__init__(client, mass, height)

        time.sleep(2)
        self._state = (self._q, self._qd)
        self._updater.start()

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    def dynamic_model(self, total_mass, height):
        model = rbdl.Model()
        return model

    def fk(self):
        fk = {}

    def update_state(self, q, qd):
        pass
