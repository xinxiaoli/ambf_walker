import os.path

import math

import numpy as np
import pandas as pd
from math import pi


class Read_Mocap(object):
    """
    Reads in the mocap data
    """

    def __init__(self, file):
        """

        :param file: path to trajectory data
        """

        self.file = file
        self.df = pd.read_csv(self.file)
        self.count = 0
        self.q_last = None

    def reset(self):
        """
        restart the line read to zero
        :return:
        """
        self.count = 0

    def get_next_point(self,dt):
        """

        :param dt: time step between the joint steps
        :return: joint angle and joint angular velocity
        """
        offset = 1
        if self.count >= len(self.df["RKneeAngles"]):
            self.count = 0
        q = np.zeros(7)
        q[0] = 0  # math.radians(float(self.df["LHipAngles"][self.count]))
        q[0 + offset] = math.radians(float(self.df["LHipAngles"][self.count]))
        q[1 + offset] = math.radians(float(self.df["LKneeAngles"][self.count]))
        q[2 + offset] = math.radians(float(self.df["LAnkleAngles"][self.count]))
        q[3 + offset] = -math.radians(float(self.df["RHipAngles"][self.count]))
        q[4 + offset] = math.radians(float(self.df["RKneeAngles"][self.count]))
        q[5 + offset] = math.radians(float(self.df["RAbsAnkleAngle"][self.count]))

        if self.count == 0:
            qd = np.zeros(len(q))
        else:
            qd = (self.q_last - q)/dt

        self.count += 1
        self.q_last = q

        return q, qd