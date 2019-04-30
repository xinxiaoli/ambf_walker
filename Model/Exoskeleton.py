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
        self._q = np.asarray([0]*6)
        self._qd = np.asarray([0]*6)
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

        mass_percentage["body"] = 0.2010
        mass_percentage["hip"] = 0.4346
        mass_percentage["thigh"] = 0.1416
        mass_percentage["shank"] = 0.0433
        mass_percentage["foot"] = 0.0137

        com_location["body"] = 0.1185
        com_location["hip"] = 0.3469
        com_location["thigh"] = 0.2425
        com_location["shank"] = 0.2529
        com_location["foot"] = 0.0182
        segments = segments = ["thigh", "shank", "foot"]
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

        mass["body"] = mass_percentage["body"] * total_mass
        mass["hip"] = mass_percentage["hip"] * total_mass
        mass["thigh"] = mass_percentage["thigh"] * total_mass
        mass["shank"] = mass_percentage["body"] * total_mass
        mass["foot"] = mass_percentage["foot"] * total_mass

        length["body"] = 0.72 * height
        length["hip"] = (30.0 / 100.0) * height
        length["thigh"] = 0.245 * height
        length["shank"] = 0.246 * height
        length["foot"] = 0.152 * height

        com["body"] = 0.1185 * length["hip"] * np.array([0.0, 1.0, 0])
        com["hip"] = 0.3469 * length["hip"] * np.array([0.0, 1.0, 0])
        com["thigh"] = 0.2425 * length["thigh"] * np.array([0.0, 1.0, 0])
        com["shank"] = 0.2529 * length["shank"] * np.array([0.0, 1.0, 0])
        com["foot"] = 0.0182 * length["foot"] * np.array([1.0, 0.0, 0])

        rgyration["body"] = np.diag([0.0970, 0.1009, 0.00825])
        rgyration["hip"] = np.diag([0.1981, 0.1021, 0.1848])
        rgyration["thigh"] = np.diag([0.1389, 0.0629, 0.1389])
        rgyration["shank"] = np.diag([0.1123, 0.0454, 0.1096])
        rgyration["foot"] = np.diag([0.0081, 0.0039, 0.0078])

        joint_location["body"] = np.array([0., length["body"], 0.0])
        joint_location["thigh"] = np.array([0., -length["hip"], 0.0])  # hip
        joint_location["shank"] = np.array([0., -length["thigh"], 0.0])  # knee
        joint_location["foot"] = np.array([0., -length["shank"], 0.0])  # ankle

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], rgyration["hip"])
        body_body = rbdl.Body.fromMassComInertia(mass["body"], com["body"], rgyration["body"])
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

        id_r = hip_id
        id_l = hip_id

        for segs in segments:
            xtrans.r = joint_location[segs]
            print segs
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

        model.gravity = np.array([0, -9.81, 0])

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
        self._q = q
        self._qd = qd
        self._state = (q, qd)
        qdd = np.zeros(self._model.qdot_size)
        rbdl.UpdateKinematics(self._model,q, qd, qdd)

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
        tau = np.asarray([0.0] * 7)
        rbdl.InverseDynamics(self._model, q_d, qd_d, qdd_d, tau)
        return tau
