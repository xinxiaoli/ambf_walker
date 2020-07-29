"""
This should be moved to a seperate repo later
"""


import abc
import numpy as np
import rbdl
import Model
import time
from GaitCore.Core import Point
from GaitCore.Core import utilities
from std_msgs.msg import Float32MultiArray
from threading import Thread
from . import Model
from GaitCore.Bio import Leg, Joint

class Exoskeleton(Model.Model):

    def __init__(self, client, mass, height):

        super(Exoskeleton, self).__init__(client)
        self.q = 7 * [0.0]
        self.qd = 7 * [0.0]
        self._handle = self._client.get_obj_handle('Hip')
        time.sleep(2)
        self._mass = mass
        self._height = height
        self._model = self.dynamic_model()
        left_joints = {}
        right_joints = {}

        for joint in (left_joints, right_joints):
            for output in ["Hip", "Knee", "Ankle"]:
                angle = Point.Point(0, 0, 0)
                force = Point.Point(0, 0, 0)
                moment = Point.Point(0, 0, 0)
                power = Point.Point(0, 0, 0)
                joint[output] = Joint.Joint(angle, moment, power, force)

        self._left_leg = Leg.Leg(left_joints["Hip"], left_joints["Knee"], left_joints["Ankle"])
        self._right_leg = Leg.Leg(right_joints["Hip"], right_joints["Knee"], right_joints["Ankle"])

        self._state = (self._q, self._qd)
        self._updater.start()

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q[0:6], self.qd[0:6], qdd[0:6], tau)
        return tau

    def dynamic_model(self):
        # add in mass and height params
        model = rbdl.Model()
        bodies = {}
        mass = {}
        com = {}
        inertia = {}
        bodies["right"] = {}
        bodies["left"] = {}
        segments = ["thigh", "shank", "foot"]

        mass["hip"] = 2.37
        mass["right_thigh"] = 2.11
        mass["left_thigh"] = 2.11
        mass["right_shank"] = 1.28
        mass["left_shank"] = 1.28
        mass["right_foot"] = 0.86
        mass["left_foot"] = 0.86
        parent_dist = {}
        parent_dist["hip"] = np.array([0.0, 0.0, 0.0])

        parent_dist["left_thigh"] = np.array([0.237, -0.124, -0.144])
        parent_dist["left_shank"] = np.array([0.033, -0.03, -0.436])
        parent_dist["left_foot"] = np.array([0.02, -0.027, -0.39])

        parent_dist["right_thigh"] = np.array([-0.237, -0.124,  -0.144])
        parent_dist["right_shank"] = np.array([0.033, -0.03,  -0.436])
        parent_dist["right_foot"] = np.array([0.02, -0.027,  -0.39])

        inertia["hip"] = np.diag([ 0.0,0.0,0.0])

        inertia["left_thigh"] = np.diag([0.0, 0.0, 0.07])
        inertia["left_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["left_foot"] = np.diag([0.07, 0.07, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.00, 0.07])
        inertia["right_shank"] = np.diag([0.18, 0.18, 0.0])
        inertia["right_foot"] = np.diag([0.07, 0.07, 0.0])

        com["hip"] = np.array([0.00, -0.02, 0.18])
        com["left_thigh"] = np.array([0.02, 0.01,  -0.09])
        com["left_shank"] = np.array([-0.02, -0.007, 0.06])
        com["left_foot"] = np.array([0.08, -0.06, 0.04])

        com["right_thigh"] = np.array([-0.02, 0.01, -0.09])
        com["right_shank"] = np.array([0.02, -0.007, 0.06])
        com["right_foot"] = np.array([0.08, -0.06, 0.04])


        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs], inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs], inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E = np.eye(3)

        self.hip = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body,"hip")
        joint_rot_z =  rbdl.Joint.fromJointType("JointTypeRevoluteX")

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

        # constraint_set_right = rbdl.ConstraintSet()
        # constraint_set_left = rbdl.ConstraintSet()
        # constraint_set_both = rbdl.ConstraintSet()
        #
        # constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        # constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")
        #
        # constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        # constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")
        #
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")
        # constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 0., 1.]), "right_heel_z")
        #
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")
        # constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 0., 1.]), "left_heel_z")
        #
        # constraint_set_right.Bind(model)
        # constraint_set_left.Bind(model)
        # constraint_set_both.Bind(model)

        return model

    def fk(self):

        fk = {}

        point_local = np.array([0.0, 0.0, 0.0])

        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_thigh, point_local)
        fk["left_hip"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_shank, point_local)
        fk["left_knee"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_foot, point_local)
        fk["left_ankle"] = Point.Point(data[0], data[1], data[2])

        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_thigh, point_local)
        fk["right_hip"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_shank, point_local)
        fk["right_knee"] = Point.Point(data[0], data[1], data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_foot, point_local)
        fk["right_ankle"] = Point.Point(data[0], data[1], data[2])

        q_left = self.get_left_leg().ankle.angle.z
        q_right = self.get_right_leg().ankle.angle.z
        fk["left_toe"] = Point.Point(0, 0, 0)
        fk["left_toe"].x = fk["left_ankle"].x - 0.8 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_toe"].y = fk["left_ankle"].y - 0.8 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_toe"].z = fk["left_ankle"].z - 0.05 + 0.8 * (8.0 / 100.0) * self._height * np.sin(q_left)

        fk["left_heel"] = Point.Point(0,0,0)
        fk["left_heel"].x = fk["left_ankle"].x + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_heel"].y = fk["left_ankle"].y + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_left)
        fk["left_heel"].z = fk["left_ankle"].z - 0.05 + 0.2 * (8.0 / 100.0) * self._height * np.sin(q_left)

        fk["right_toe"] = Point.Point(0, 0, 0)
        fk["right_toe"].x = fk["right_ankle"].x - 0.8 * (8.0 / 100.0) * 1.57 * np.cos(q_right)
        fk["right_toe"].y = fk["right_ankle"].y - 0.8 * (8.0 / 100.0) * 1.57 * np.cos(q_right)
        fk["right_toe"].z = fk["right_ankle"].z - 0.05 + 0.8 * (8.0 / 100.0) * self._height * np.sin(q_right)

        fk["right_heel"] = Point.Point(0, 0, 0)
        fk["right_heel"].x = fk["right_ankle"].x + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_right)
        fk["right_heel"].y = fk["right_ankle"].y + 0.2 * (8.0 / 100.0) * self._height * np.cos(q_right)
        fk["right_heel"].z = fk["right_ankle"].z - 0.05 + 0.2 * (8.0 / 100.0) * self._height * np.sin(q_right)

        return fk


    def stance_trajectory(self, tf=2, dt=0.01):

        hip = Model.get_traj(0.0, -0.3, 0.0, 0.0, tf, dt)
        knee = Model.get_traj(0.0, 0.20, 0.0, 0., tf, dt)
        ankle = Model.get_traj(-0.349, 0.157 + 0.1, 0.0, 0.0, tf, dt)
        return hip, knee, ankle


    def update_state(self, q, qd):
        self.get_left_leg().hip.angle.z = q[0]
        self.get_left_leg().knee.angle.z = q[1]
        self.get_left_leg().ankle.angle.z = q[2]

        self.get_right_leg().hip.angle.z = q[3]
        self.get_right_leg().knee.angle.z = q[4]
        self.get_right_leg().ankle.angle.z = q[5]


