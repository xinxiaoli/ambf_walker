"""
This should be moved to a seperate repo later
"""


import abc
import numpy as np
import rbdl

class Exoskeleton(object):

    def __init__(self, mass, height):
        self._mass = mass
        self._height = height
        self._model = self.dynamic_model(mass, height)
        self.joint_order = ["left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle"]
        self._q = np.asarray([0]*7)
        self._qd = np.asarray([0]*7)
        self._state = (self._q, self._qd)

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

    def dynamic_model(self, total_mass, height):

        mass_percentage = {}
        com_location = {}
        mass = {}
        com = {}
        length = {}
        joint_location = {}
        rgyration = {}


        com_location["body"] = 0.1185
        com_location["hip"] = 0.3469
        com_location["thigh"] = 0.2425
        com_location["shank"] = 0.2529
        com_location["foot"] = 0.0182
        segments = ["thigh", "shank", "foot"]
        #joint_rot_z = rbdl.Joint.fromJointType("JointTypeRevoluteZ")
        a = np.array([0., 0., -1., 0., 0., 0.])
        joint_rot_z = rbdl.Joint.fromJointAxes([a])

        floating_base = rbdl.Joint.fromJointType("JointTypeFloatingBase")
        ztrans = rbdl.SpatialTransform()
        ztrans.r = np.array([0., 0, -1.])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0., 1., 0.])

        model = rbdl.Model()
        bodies = {}
        bodies["right"] = {}
        bodies["left"] = {}

        mass["body"] = 10
        mass["hip"] = 2.3677
        mass["thigh"] = 2.1138
        mass["shank"] = 1.2804
        mass["foot"] = 0.85523

        parent_dist = [[0.0, 0.0, 0.0], #base
                       [0.22, -0.125, -0.141], # hip -> left Thigh
                       [0.22, 0.125, 0.141], # hip -> right Thigh
                       [0.0, -0.04, -0.3], # right thigh -> right shank
                       [0.0, -0.04, 0.3], # Left thigh -> Left shank:
                       [0.017, 0.021, -0.137], # Right shank -> Right foot
                       [0.017, 0.021, -0.137]] # Left shank -> Left foot:

        parent_dist = np.array(parent_dist)

        com["hip"] = np.array([0.235, .372, 1.474])
        com["left_thigh"] = np.array([ 0.473, 0.248, 1.2 ])
        com["left_shank"] = np.array([0.497, 0.254,  0.641])
        com["left_foot"] = np.array([ 0.475, 0.26,  0.432])

        com["right_thigh"] = np.array([ 0.473, 0.248, 1.2 ])
        com["right_shank"] = np.array([0.497, 0.254,  0.641])
        com["right_foot"] = np.array([ 0.0, 0.254,  0.432])


        rgyration["body"] = np.diag([0.0, 1.0, 0.0])
        rgyration["hip"] = np.diag([0.0, -0.023, 0.184])
        rgyration["thigh"] = np.diag([0.0226, 0.015, -0.086])
        rgyration["shank"] = np.diag([0.024, -0.0007, 0.057])
        rgyration["foot"] = np.diag([0.077, 0.06, -0.04])

        joint_location["body"] = np.array([0., length["body"], 0.0])
        joint_location["thigh"] = np.array([0., -length["hip"], 0.0])  # hip
        joint_location["shank"] = np.array([0., -length["thigh"], 0.0])  # knee
        joint_location["foot"] = np.array([0., -length["shank"], 0.0])  # ankle

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], rgyration["hip"])
        for segs in segments:
            bodies["right"][segs] = rbdl.Body.fromMassComInertia(mass[segs], com[segs], rgyration[segs])
            bodies["left"][segs] = rbdl.Body.fromMassComInertia(mass[segs], com[segs], rgyration[segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = joint_location["body"]

        hip_id = model.AppendBody(xtrans,
                                  joint_rot_z,
                                  hip_body,
                                  "hip"
                                  )

        id_l = hip_id
        id_r = hip_id

        for segs in segments:
            xtrans.r = joint_location[segs]
            id_l = model.AddBody(id_l,
                                 xtrans,
                                 joint_rot_z,
                                 bodies["left"][segs],
                                 "left_" + segs
                                 )

        for segs in segments:
            xtrans.r = joint_location[segs]
            id_r = model.AddBody(id_r,
                                 xtrans,
                                 joint_rot_z,
                                 bodies["right"][segs],
                                 "right_" + segs
                                 )

        heel_point = np.array([-0.05, -0.2 * length["foot"], 0.])
        medial_point = np.array([-0.05, 0.8 * length["foot"], 0.])

        model.gravity = np.array([0, 0, -9.81])

        constraint_set_right = rbdl.ConstraintSet()
        constraint_set_left = rbdl.ConstraintSet()
        constraint_set_both = rbdl.ConstraintSet()

        constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        constraint_set_right.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")

        constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        constraint_set_left.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")

        constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([1., 0., 0.]), "right_heel_x")
        constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 1., 0.]), "right_heel_y")
        constraint_set_both.AddContactConstraint(id_r, heel_point, np.array([0., 0., 1.]), "right_heel_z")

        constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([1., 0., 0.]), "left_heel_x")
        constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 1., 0.]), "left_heel_y")
        constraint_set_both.AddContactConstraint(id_l, heel_point, np.array([0., 0., 1.]), "left_heel_z")

        constraint_set_right.Bind(model)
        constraint_set_left.Bind(model)
        constraint_set_both.Bind(model)

        return model

    def update_joints(self, q, qd):
        """

        :param q:
        :param qd:
        :return:
        """

        # 0   1  2  3  4  5  6
        # -  LH RH LK RK LA RA
        # --->
        # -  LH LK LA RH RK RA

        self._q = self.ambf_to_dyn(q) #np.asarray( [q[0], q[1], q[3], q[2], q[4], q[6], q[5]])
        self._qd = self.ambf_to_dyn(qd) #np.asarray( [qd[0], qd[1], qd[3], qd[2], qd[4], qd[6], qd[5]])
        self._state = (self._q, self._qd)
        qdd = np.zeros(self._model.qdot_size)
        rbdl.UpdateKinematics(self._model, self._q, self._qd, qdd)

    def  ambf_to_dyn(self, q):
        return  np.asarray( [q[0], q[1], q[3], q[2], q[4], q[6], q[5]])

    def make_foot(self, left_ankle, right_ankle):

        foot = {}
        # TODO put the real nums in

        foot["left_toe"] = {}
        foot["left_heel"] = {}
        foot["right_toe"] = {}
        foot["right_heel"] = {}

        foot["left_toe"]["x"] = left_ankle["x"] + 0.8 * (4.25 / 100.0) * self._height * np.cos(-self._q[3])
        foot["left_toe"]["y"] = left_ankle["y"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["left_heel"]["x"] = left_ankle["x"] - 0.2 * (4.25 / 100.0) * self._height * np.cos(-self._q[3])
        foot["left_heel"]["y"] = left_ankle["y"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["right_toe"]["x"] = right_ankle["x"] + 0.8 * (4.25 / 100.0) * 1.57 * np.cos(-self._q[6])
        foot["right_toe"]["y"] = right_ankle["y"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])
        foot["right_heel"]["x"] = right_ankle["x"] - 0.2 * (4.25 / 100.0) * self._height * np.cos(-self._q[6])
        foot["right_heel"]["y"] = right_ankle["y"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])

        return foot

    @property
    def fk(self):
        fk = {}
        for index, joint in enumerate(self.joint_order):
            point = {}
            point["x"] = self._model.X_base[2 + index].r[0]
            point["y"] = self._model.X_base[2 + index].r[1]
            point["z"] = self._model.X_base[2 + index].r[2]
            fk[joint] = point

        foot = self.make_foot(fk["left_ankle"], fk["right_ankle"])
        fk.update(foot)

        return fk

    def calculate_dynamics(self, q_d, qd_d, qdd_d):
        q = self.ambf_to_dyn(q_d)
        qd = self.ambf_to_dyn(qd_d)
        qdd = self.ambf_to_dyn(qdd_d)
        tau = np.asarray([0.0] * 7)
        rbdl.InverseDynamics(self._model, q, qd, qdd, tau)
        return self.ambf_to_dyn(tau)
