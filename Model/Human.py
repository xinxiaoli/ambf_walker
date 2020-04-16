"""
This should be moved to a seperate repo later
"""


import abc
import numpy as np
import rbdl
import Model
import time
from lib.GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from threading import Thread

class Human(Model.Model):

    def __init__(self, client, mass, height):
        # inits dynamic model and joints for leg
        super(Human, self).__init__(client, mass, height)

        self.handle = self._client.get_obj_handle('body')
        self.body_children = ["left_thigh", "left_foot", "left_calf", "left_arm_bot", "left_arm_top", "left_hand",
                         "right_thigh", "right_foot", "right_calf", "right_arm_bot", "right_arm_top", "right_hand",
                         "head"]

        # num_of_segments should be initialized with the dynamical model, which is created in the constructor
        self.q = self.num_of_segments * [0.0]
        self.qd = self.num_of_segments * [0.0]

        time.sleep(2)
        self._state = (self._q, self._qd)
        self._updater.start()   # start update thread

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    @q.setter
    def q(self, value):
        # TODO: transform RBDL to AMBF
        self._q = np.asarray(value)

    @qd.setter
    def qd(self, value):
        # TODO: transform RBDL to AMBF
        value[2] *= -1
        value[5] *= -1
        self._qd = np.asarray(value)


    def dynamic_model(self, total_mass, height):
        # TODO: Add joints and bodies for arms and head

        model = rbdl.Model()
        bodies = {}
        parent_dist = {}
        mass = {}
        com = {}
        inertia = {}
        bodies["right"] = {}
        bodies["left"] = {}
        segments = ["thigh", "calf", "foot"]

        # percent total body weight from average in de Leva
        per_head = 6.81
        per_trunk = 43.02
        per_upper_arm = 2.63
        per_lower_arm = 1.5
        per_hand = 0.585
        per_thigh = 14.47
        per_calf = 4.57
        per_foot = 1.33

        mass["head"] = total_mass * per_head
        mass["body"] = total_mass * per_trunk
        mass["right_arm_top"] = total_mass * per_upper_arm
        mass["left_arm_top"] = total_mass * per_upper_arm
        mass["right_arm_bot"] = total_mass * per_lower_arm
        mass["left_arm_bot"] = total_mass * per_lower_arm
        mass["right_hand"] = total_mass * per_hand
        mass["left_hand"] = total_mass * per_hand
        mass["right_thigh"] = total_mass * per_thigh
        mass["left_thigh"] = total_mass * per_thigh
        mass["right_calf"] = total_mass * per_calf
        mass["left_calf"] = total_mass * per_calf
        mass["right_foot"] = total_mass * per_foot
        mass["left_foot"] = total_mass * per_foot

        parent_dist["body"] = np.array([0.0, 0.0, 0.0])

        parent_dist["left_thigh"] = np.array([0.237, -0.124, -0.144])
        parent_dist["left_calf"] = np.array([0.033, -0.03, -0.436])
        parent_dist["left_foot"] = np.array([0.02, -0.027, -0.39])

        parent_dist["right_thigh"] = np.array([-0.237, -0.124, -0.144])
        parent_dist["right_calf"] = np.array([0.033, -0.03, -0.436])
        parent_dist["right_foot"] = np.array([0.02, -0.027, -0.39])

        self.num_of_segments = len(parent_dist)

        inertia["body"] = np.diag([0.0, 0.0, 0.0])

        inertia["left_thigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["left_calf"] = np.diag([0.18, 0.18, 0.0])
        inertia["left_foot"] = np.diag([0.07, 0.07, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["right_calf"] = np.diag([0.18, 0.18, 0.0])
        inertia["right_foot"] = np.diag([0.07, 0.07, 0.0])

        com["body"] = np.array([0.00, -0.02, 0.18])

        com["left_thigh"] = np.array([0.02, 0.01, -0.09])
        com["left_calf"] = np.array([-0.02, -0.007, 0.06])
        com["left_foot"] = np.array([0.08, -0.06, 0.04])

        com["right_thigh"] = np.array([-0.02, 0.01, -0.09])
        com["right_calf"] = np.array([0.02, -0.007, 0.06])
        com["right_foot"] = np.array([0.08, -0.06, 0.04])

        body_rbdl = rbdl.Body.fromMassComInertia(mass["body"], com["body"], inertia["body"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs],
                                                                   inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs],
                                                                  inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        self.body = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), body_rbdl, "hip")
        joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteX")

        xtrans.r = parent_dist["left_thigh"]
        self.left_thigh = model.AddBody(self.body, xtrans, joint_rot_z, bodies["left_thigh"], "left_thigh")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_calf"]
        self.left_calf = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_calf"], "left_calf")

        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_calf, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")

        xtrans.r = parent_dist["right_thigh"]
        self.right_thigh = model.AddBody(self.body, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_calf"]
        self.right_calf = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_calf"], "right_calf")

        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_calf, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")

        model.gravity = np.array([0, 0, -9.81])

        return model

    def fk(self):

        fk = {}

    def calculate_dynamics(self, qdd):
        # Overrides as human has more states. Not sure if really need since upperbody will not be controlled?
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q, self.qd, qdd, tau)
        return tau
