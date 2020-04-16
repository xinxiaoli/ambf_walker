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

        self._handle = self._client.get_obj_handle('body')

        """
        state of each segment, format in order of creation:
        right thigh
        left thigh
        right shin
        left shin
        right foot
        left foot
        body
        """
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

    def dynamic_model(self, total_mass, height):
        # TODO: Add joints and bodies for arms and head

        model = rbdl.Model()
        bodies = {}
        mass = {}
        com = {}
        inertia = {}
        bodies["right"] = {}
        bodies["left"] = {}
        segments = ["thigh", "shank", "foot"]

        # percent total body weight from average in de Leva
        per_head = 6.81
        per_trunk = 43.02
        per_upper_arm = 2.63
        per_lower_arm = 1.5 + 0.585     # forearm + hand
        per_thigh = 14.47
        per_shank = 4.57
        per_foot = 1.33

        mass["head"] = total_mass * per_head
        mass["body"] = total_mass * per_trunk
        mass["right_arm_top"] = total_mass * per_upper_arm
        mass["left_arm_top"] = total_mass * per_upper_arm
        mass["right_arm_bot"] = total_mass * per_lower_arm
        mass["left_arm_bot"] = total_mass * per_lower_arm
        mass["right_thigh"] = total_mass * per_thigh
        mass["left_thigh"] = total_mass * per_thigh
        mass["right_shank"] = total_mass * per_shank
        mass["left_shank"] = total_mass * per_shank
        mass["right_foot"] = total_mass * per_foot
        mass["left_foot"] = total_mass * per_foot

        parent_dist = {}
        parent_dist["body"] = np.array([0.0, 0.0, 0.0])

        parent_dist["left_thigh"] = np.array([0.237, -0.124, -0.144])
        parent_dist["left_shank"] = np.array([0.033, -0.03, -0.436])
        parent_dist["left_foot"] = np.array([0.02, -0.027, -0.39])

        parent_dist["right_thigh"] = np.array([-0.237, -0.124, -0.144])
        parent_dist["right_shank"] = np.array([0.033, -0.03, -0.436])
        parent_dist["right_foot"] = np.array([0.02, -0.027, -0.39])

        self.num_of_segments = len(parent_dist)

        inertia["hip"] = np.diag([0.0, 0.0, 0.0])

        inertia["left_thigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["left_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["left_foot"] = np.diag([0.07, 0.07, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["right_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["right_foot"] = np.diag([0.07, 0.07, 0.0])

        com["hip"] = np.array([0.00, -0.02, 0.18])
        com["left_thigh"] = np.array([0.02, 0.01, -0.09])
        com["left_shank"] = np.array([-0.02, -0.007, 0.06])
        com["left_foot"] = np.array([0.08, -0.06, 0.04])

        com["right_thigh"] = np.array([-0.02, 0.01, -0.09])
        com["right_shank"] = np.array([0.02, -0.007, 0.06])
        com["right_foot"] = np.array([0.08, -0.06, 0.04])

        hip_body = rbdl.Body.fromMassComInertia(mass["body"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs],
                                                                   inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs],
                                                                  inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        self.hip = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body, "hip")
        joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteX")

        xtrans.r = parent_dist["left_thigh"]
        self.left_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["left_thigh"], "left_thigh")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_shank"]
        self.left_shank = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_shank"], "left_shank")

        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_shank, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")

        xtrans.r = parent_dist["right_thigh"]
        self.right_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")

        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_shank"]
        self.right_shank = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_shank"], "right_shank")

        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_shank, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")

        model.gravity = np.array([0, 0, -9.81])

        return model

    def fk(self):

        fk = {}

    def calculate_dynamics(self, qdd):
        # Overrides as human has more states. Not sure if really need since upperbody will not be controlled?
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q[0:6], self.qd[0:6], qdd[0:6], tau)
        return tau

    def update_state(self, q, qd):
        self.get_left_leg().hip.angle.z = q[0]
        self.get_left_leg().knee.angle.z = q[1]
        self.get_left_leg().ankle.angle.z = q[2]

        self.get_right_leg().hip.angle.z = q[3]
        self.get_right_leg().knee.angle.z = q[4]
        self.get_right_leg().ankle.angle.z = q[5]
