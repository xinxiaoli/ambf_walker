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

        # num_of_segments should be initialized with the dynamical model, which is created in the constructor
        self.q = self.num_of_segments * [0.0]
        self.qd = self.num_of_segments * [0.0]

        time.sleep(2)
        self._state = (self._q, self._qd)
        self._updater.start()  # start update thread

        """
        Order of AMBF Joints        Order of RBDL Joints
        left hip                    left hip
        left ankle                  left knee
        left knee                   left ankle
        left elbow                  right hip
        left shoulder               right knee
        left arm bot-left hand      right ankle
        neck                        left shoulder
        right hip                   left elbow
        right ankle                 left wrist
        right knee                  right shoulder
        right elbow                 right elbow
        right shoulder              right wrist
        right wrist                 neck
        """

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
        segments = ["thigh", "calf", "foot", "arm_bot", "arm_top"]

        # percent total body weight from average in de Leva
        per_head = 6.81 / 100
        per_trunk = 43.02 / 100
        per_upper_arm = 2.63 / 100
        per_lower_arm = 1.5 / 100
        per_hand = 0.585 / 100
        per_thigh = 14.47 / 100
        per_calf = 4.57 / 100
        per_foot = 1.33 / 100

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
        parent_dist["head"] = np.array([0.00002, -0.006489, 0.261994])

        parent_dist["left_thigh"] = np.array([0.066515, -0.028853, -0.388835])
        parent_dist["left_calf"] = np.array([[0.05359, 0.00073, 0.40753]])
        parent_dist["left_foot"] = np.array([0.0, 0.00623, -0.41995])

        parent_dist["right_thigh"] = np.array([-0.066515, -0.028853, -0.388835])
        parent_dist["right_calf"] = np.array([-0.05359, 0.00073, -0.40752])
        parent_dist["right_foot"] = np.array([0.0, -0.00623, -0.41995])

        parent_dist["left_arm_top"] = np.array([0.21473, 0.00711, 0.15701])
        parent_dist["left_arm_bot"] = np.array([0.13306, -0.04375, -0.24511])
        parent_dist["left_hand"] = np.array([-0.76922, -0.09107, -0.2435 ])

        parent_dist["right_arm_top"] = np.array([-0.21473, 0.00711, 0.15701])
        parent_dist["right_arm_bot"] = np.array([-0.12709, -0.04443, -0.24723])
        parent_dist["right_hand"] = np.array([-0.07497, -0.09137, -0.24297])

        self.num_of_segments = len(parent_dist)

        # Inertial Matrices for each segment
        inertia["body"] = np.diag([0.092407, 0.032753, 0.0]) * mass["body"]
        inertia["head"] = np.diag([0.196531, 0.162497, 0.257624]) * mass["head"]

        inertia["left_thigh"] = np.diag([0.433321, 0.435854, 0.0]) * mass["left_thigh"]
        inertia["left_calf"] = np.diag([0.05865, 0.0, 0.0]) * mass["left_calf"]
        inertia["left_foot"] = np.diag([0.174576, 0.173107, 0.0]) * mass["left_foot"]

        inertia["right_thigh"] = np.diag([-0.3066, -0.3041, 0.0309]) * mass["right_thigh"]  # needs to be fixed in blender
        inertia["right_calf"] = np.diag([0.058648, 0.0, 0.0]) * mass["right_calf"]
        inertia["right_foot"] = np.diag([0.174573, 0.173105, 0.0]) * mass["right_foot"]

        inertia["left_arm_top"] = np.diag([0.240887, 0.218034, 0.0]) * mass["left_arm_top"]
        inertia["left_arm_bot"] = np.diag([0.243486, 0.243336, 0.026448]) * mass["left_arm_bot"]
        inertia["left_hand"] = np.diag([0.14476, 0.148051, 0.020571]) * mass["left_hand"]

        inertia["right_arm_top"] = np.diag([0.240887, 0.218034, 0.0]) * mass["right_arm_top"]
        inertia["right_arm_bot"] = np.diag([-0.2124, -0.2126, -0.0228]) * mass["right_arm_bot"]  # needs to be fixed in blender
        inertia["right_hand"] = np.diag([0.144774, 0.148058, 0.020569]) * mass["right_hand"]

        # Center of masses
        com["body"] = np.array([0.001275, -0.09677, 0.006332])
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
        # xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_calf"]
        self.left_calf = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_calf"], "left_calf")

        # Left Ankle
        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_calf, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")

        # Right Hip
        xtrans.r = parent_dist["right_thigh"]
        self.right_thigh = model.AddBody(self.body, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")

        # Right Knee
        # xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_calf"]
        self.right_calf = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_calf"], "right_calf")

        # Right Ankle
        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_calf, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")

        # Left Shoulder

        # Left Elbow

        # Left Wrist

        # Right Shoulder

        # Right Elbow

        # Right Wrist

        # Neck

        model.gravity = np.array([0, 0, -9.81])

        return model

    def fk(self):
        fk = {}

    def calculate_dynamics(self, qdd):
        # Overrides as human has more states. Not sure if really need since upperbody will not be controlled?
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q, self.qd, qdd, tau)
        return tau

    def ambf_to_rbdl(self, input_q, reverse = false):
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
        # order of rbdl wrt ambf
        rbdl_order = [1, 3, 2, 8, 10, 9, 5, 4, 6, 12, 11, 13, 7]
        # for now, this order is included since we don't have the arm joints yet
        rbdl_order_lower = rbdl_order[:7]

        order = rbdl_order_lower
        transformed_q = len(order)

        for i in range(len(order)):
            if not reverse:
                transformed_q[i] = input_q[order[i]]
            else:
                transformed_q[order[i]] = input_q[i]

        return transformed_q