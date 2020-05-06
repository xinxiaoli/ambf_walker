"""
This should be moved to a seperate repo later
"""

import abc
import numpy as np
import rbdl
import Model
import time
import math
from lib.GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from threading import Thread


class Human(Model.Model):

    def __init__(self, client, mass, height):
        # inits dynamic model and joints for leg
        super(Human, self).__init__(client, mass, height)

        self.handle = self._client.get_obj_handle('body')
        self.mass = mass

        # num_of_segments should be initialized with the dynamical model, which is created in the constructor
        self.num_joints = len(self.handle.get_joint_names())
        self.q = self.num_joints * [0.0]
        self.qd = self.num_joints * [0.0]
        self.fext = self.num_joints * [rbdl.SpatialVector()]

        time.sleep(2)
        self._state = (self._q, self._qd)
        self._updater.start()  # start update thread

        # self.joint_name = self.handle.get_joint_names()

        self.ambf_order_crutch_left = {'crutch': 0, 'hip': 1, 'ankle': 2, 'knee': 3, 'elbow': 4, 'shoulder': 5,
                                       'wrist': 6, 'neck': 7}
        self.ambf_order_crutch_right = {'crutch': 8, 'hip': 9, 'ankle': 10, 'knee': 11, 'elbow': 12, 'shoulder': 13,
                                        'wrist': 14, 'neck': 7}

        self.joint_limits = self.get_limits()

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = np.concatenate(value)

    # @q.setter
    # def q(self, value):
    #     # TODO: transform AMBF to RBDL
    #     self._q = np.asarray(value)
    #
    # @qd.setter
    # def qd(self, value):
    #     # TODO: transform RBDL to AMBF
    #     value[2] *= -1
    #     value[5] *= -1
    #     self._qd = np.asarray(value)

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

        segments = ["thigh", "calf", "foot", "arm_bot", "arm_top", "hand"]

        # percent total body weight from average in de Leva
        per_head = 6.81 / 100.0
        per_trunk = 43.02 / 100.0
        per_upper_arm = 2.63 / 100.0
        per_lower_arm = 1.5 / 100.0
        per_hand = 0.585 / 100.0
        per_thigh = 14.47 / 100.0
        per_calf = 4.57 / 100.0
        per_foot = 1.33 / 100.0

        # per_head = 1
        # per_trunk = 1
        # per_upper_arm = 1
        # per_lower_arm = 1
        # per_hand = 1
        # per_thigh = 1
        # per_calf = 1
        # per_foot = 1

        # Masses for each of the segments based off of percents
        mass["head"] = total_mass * per_head
        mass["body"] = total_mass * per_trunk

        mass["left_thigh"] = total_mass * per_thigh
        mass["left_calf"] = total_mass * per_calf
        mass["left_foot"] = total_mass * per_foot

        mass["right_thigh"] = total_mass * per_thigh
        mass["right_calf"] = total_mass * per_calf
        mass["right_foot"] = total_mass * per_foot

        mass["left_arm_top"] = total_mass * per_upper_arm
        mass["left_arm_bot"] = total_mass * per_lower_arm
        mass["left_hand"] = total_mass * per_hand

        mass["right_arm_top"] = total_mass * per_upper_arm
        mass["right_arm_bot"] = total_mass * per_lower_arm
        mass["right_hand"] = total_mass * per_hand

        # Distance of base of link from its parent
        parent_dist["body"] = np.array([0.0, 0.0, 0.0])
        parent_dist["head"] = np.array([0.00002, -0.00024, 0.29875])

        parent_dist["left_thigh"] = np.array([0.06652004, -0.0226, -0.35208])
        parent_dist["left_calf"] = np.array([0.05359, 0.00073, 0.40753])
        parent_dist["left_foot"] = np.array([0.0, 0.00623, -0.41995])

        parent_dist["right_thigh"] = np.array([-0.06652004, -0.0226, -0.35208])
        parent_dist["right_calf"] = np.array([-0.05359, 0.00073, -0.40752])
        parent_dist["right_foot"] = np.array([0.0, -0.00623, -0.41995])

        parent_dist["left_arm_top"] = np.array([0.21473404, 0.013356, 0.193767])
        parent_dist["left_arm_bot"] = np.array([0.13306, -0.04375, -0.24511])
        parent_dist["left_hand"] = np.array([-0.76922, -0.09107, -0.2435])

        parent_dist["right_arm_top"] = np.array([-0.21473404, 0.013356, 0.193767])
        parent_dist["right_arm_bot"] = np.array([-0.12709, -0.04443, -0.24723])
        parent_dist["right_hand"] = np.array([-0.07497, -0.09137, -0.24297])

        self.num_of_segments = len(parent_dist)

        # Inertial Matrices for each segment
        inertia["body"] = np.diag([0.0716636, 0.0203119, 0.0]) * mass["body"]
        inertia["head"] = np.diag([0.196531, 0.162497, 0.257624]) * mass["head"]

        inertia["left_thigh"] = np.diag([0.433321, 0.435854, 0.0]) * mass["left_thigh"]
        inertia["left_calf"] = np.diag([0.05865, 0.0, 0.0]) * mass["left_calf"]
        inertia["left_foot"] = np.diag([0.174576, 0.173107, 0.0]) * mass["left_foot"]

        inertia["right_thigh"] = np.diag([0.433321, 0.435854, 0.0]) * mass[
            "right_thigh"]  # needs to be fixed in blender
        inertia["right_calf"] = np.diag([0.058648, 0.0, 0.0]) * mass["right_calf"]
        inertia["right_foot"] = np.diag([0.174573, 0.173105, 0.0]) * mass["right_foot"]

        inertia["left_arm_top"] = np.diag([0.240887, 0.218034, 0.0]) * mass["left_arm_top"]
        inertia["left_arm_bot"] = np.diag([0.243486, 0.243336, 0.026448]) * mass["left_arm_bot"]
        inertia["left_hand"] = np.diag([0.14476, 0.148051, 0.020571]) * mass["left_hand"]

        inertia["right_arm_top"] = np.diag([0.240887, 0.218034, 0.0]) * mass["right_arm_top"]
        inertia["right_arm_bot"] = np.diag([0.243486, 0.243336, 0.026448]) * mass[
            "right_arm_bot"]  # needs to be fixed in blender
        inertia["right_hand"] = np.diag([0.144774, 0.148058, 0.020569]) * mass["right_hand"]

        # Center of masses
        com["body"] = np.array([0.001275, -0.060015, 0.000084])
        com["head"] = np.array([0.000363, 0.115569, 0.076097])

        com["left_thigh"] = np.array([0.042817, -0.010513, 0.18499])
        com["left_calf"] = np.array([0.000001, -0.201606, -0.025365])
        com["left_foot"] = np.array([0.000013, -0.02361, 0.079922])

        com["right_thigh"] = np.array([-0.042817, 0.010513, -0.184988])  # needs to be fixed in blender
        com["right_calf"] = np.array([0.000001, -0.201603, -0.025365])
        com["right_foot"] = np.array([0.000013, -0.023609, 0.079921])

        com["left_arm_top"] = np.array([0.015115, -0.090748, 0.098457])
        com["left_arm_bot"] = np.array([-0.006707, 0.012308, 0.113982])
        com["left_hand"] = np.array([-0.033968, 0.008122, 0.068737])

        com["right_arm_top"] = np.array([-0.008303, -0.089742, 0.097849])
        com["right_arm_bot"] = np.array([0.006707, -0.012308, -0.113982])  # needs to be fixed in blender
        com["right_hand"] = np.array([0.033968, 0.008124, 0.068738])

        body_rbdl = rbdl.Body.fromMassComInertia(mass["body"], com["body"], inertia["body"])
        bodies["head"] = rbdl.Body.fromMassComInertia(mass["head"], com["head"], inertia["head"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs],
                                                                   inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs],
                                                                  inertia["left_" + segs])

        # -----------SET JOINTS-----------
        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        # Fixed Body
        self.body = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), body_rbdl, "hip")
        joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteX")

        # Left Hip
        xtrans.r = parent_dist["left_thigh"]
        self.left_thigh = model.AddBody(self.body, xtrans, joint_rot_z, bodies["left_thigh"], "left_thigh")

        # Left Knee
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_calf"]
        self.left_calf = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_calf"], "left_calf")

        # Left Ankle
        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_calf, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")

        # Right Hip
        xtrans.r = parent_dist["right_thigh"]
        self.right_thigh = model.AddBody(self.body, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")

        # Right Knee
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_calf"]
        self.right_calf = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_calf"], "right_calf")

        # Right Ankle
        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_calf, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")

        # Left Shoulder
        xtrans.r = parent_dist["left_arm_top"]
        self.left_arm_top = model.AddBody(self.body, xtrans, joint_rot_z, bodies["left_arm_top"], "left_arm_top")

        # Left Elbow
        xtrans.r = parent_dist["left_arm_bot"]
        self.left_arm_bot = model.AddBody(self.left_arm_top, xtrans, joint_rot_z, bodies["left_arm_bot"],
                                          "left_arm_bot")

        # Left Wrist
        xtrans.r = parent_dist["left_hand"]
        self.left_hand = model.AddBody(self.left_arm_bot, xtrans, joint_rot_z, bodies["left_hand"], "left_hand")

        # Right Shoulder
        xtrans.r = parent_dist["right_arm_top"]
        self.right_arm_top = model.AddBody(self.body, xtrans, joint_rot_z, bodies["right_arm_top"], "right_arm_top")

        # Right Elbow
        xtrans.r = parent_dist["right_arm_bot"]
        self.right_arm_bot = model.AddBody(self.right_arm_top, xtrans, joint_rot_z, bodies["right_arm_bot"],
                                           "right_arm_bot")

        # Right Wrist
        xtrans.r = parent_dist["right_hand"]
        self.right_hand = model.AddBody(self.right_arm_bot, xtrans, joint_rot_z, bodies["right_hand"], "right_hand")

        # Neck
        xtrans.r = parent_dist["head"]
        self.head = model.AddBody(self.body, xtrans, joint_rot_z, bodies["head"], "head")

        model.gravity = np.array([0, 0, -9.81])

        return model

    def fk(self):
        fk = {}

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self.num_joints)

        # if not defined, set to the obj attributes
        q = self.q
        qd = self.qd

        self.calc_fext()
        rbdl.InverseDynamics(self._model, self.ambf_to_rbdl(q), self.ambf_to_rbdl(qd), self.ambf_to_rbdl(qdd), tau)
        return self.ambf_to_rbdl(tau, reverse=True)

    def ambf_to_rbdl(self, input_arr, reverse=False):
        """
        bool reverse: to transfrom rbdl to ambf
        Order of AMBF Joints        Order of RBDL Joints
        1  left hip                    left hip
        2  left ankle                  left knee
        3  left knee                   left ankle
        4  left elbow                  right hip
        5  left shoulder               right knee
        6  left arm bot-left hand      right ankle
        7  neck                        left shoulder
        8  right hip                   left elbow
        9  right ankle                 left wrist
        10 right knee                  right shoulder
        11 right elbow                 right elbow
        12 right shoulder              right wrist
        13 right wrist                 neck
        """
        """
        Order of AMBF Joints crutch    Order of RBDL Joints
        0  left crutch hip             left hip
        1  left hip                    left knee
        2  left ankle                  left ankle
        3  left knee                   right hip
        4  left elbow                  right knee
        5  left shoulder               right ankle
        6  left arm bot-left hand      left shoulder
        7  neck                        left elbow
        8  right_crutch_shoulder       left wrist
        9  right hip                   right shoulder
        10 right ankle                 right elbow
        11 right knee                  right wrist
        12 right elbow                 neck
        13 right shoulder              
        14 right wrist                 
        """

        # order of rbdl wrt ambf
        rbdl_order = [0, 2, 3, 7, 9, 10, 4, 3, 5, 11, 12, 12, 6]
        self.rbdl_order_crutch = [1, 3, 2, 9, 11, 10, 5, 4, 6, 13, 12, 14, 7]

        # for now, this order is included since we don't have the arm joints yet
        rbdl_order_lower = rbdl_order[:7]

        order = self.rbdl_order_crutch
        if not reverse:
            transformed = np.zeros(len(order))
        else:
            transformed = np.zeros(15)

        for i in range(len(order)):
            if not reverse:
                transformed[i] = input_arr[order[i]]
            else:
                transformed[order[i]] = input_arr[i]

        return transformed

    def get_limits(self):
        return {
            'hip': [0.8727, -1.5708],
            'knee': [0, 2.618],
            'ankle': [0.7854, -0.3491],
            }

    def calc_fext(self):
        """
        calculates the external forces acting on the feet

        for now assumes that both feet are on the ground and weight is distributed between
        force and moment are acting about the frame of the foot
        """
        g = 9.81
        force = g*self.mass/2

        z = self.handle.get_pos().z
        roll = self.handle.get_rpy()[0]

        r = z * math.tan(roll - math.pi/2)

        moment = force * r

        # set ankle forces in the z
        self.fext[self.ambf_order_crutch_left['ankle']][2] = force
        self.fext[self.ambf_order_crutch_right['ankle']][2] = force

        # set ankle forces in the z
        self.fext[self.ambf_order_crutch_left['ankle']][5] = moment
        self.fext[self.ambf_order_crutch_right['ankle']][5] = moment

