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

        model = rbdl.Model()
        bodies = {}
        mass = {}
        com = {}
        inertia = {}
        bodies["right"] = {}
        bodies["left"] = {}
        segments = ["thigh", "shank", "foot"]

        mass["hip"] = 2.3677
        mass["right_thigh"] = 2.1138
        mass["left_thigh"] = 2.1138
        mass["right_shank"] = 1.2804
        mass["left_shank"] = 1.2804
        mass["right_foot"] = 0.85523
        mass["left_foot"] = 0.85523
        parent_dist = {}

        parent_dist["hip"] = np.array([0.0, 0.0, 0.0])

        parent_dist["right_thigh"] = np.array([-0.237, -0.139, -0.144])
        parent_dist["right_shank"] = np.array([-0.001, 0.039, -0.301])
        parent_dist["right_foot"] = np.array([0.024, 0.031, -0.138])

        parent_dist["left_thigh"] = np.array([00.237, -0.139, -0.144])
        parent_dist["left_shank"] = np.array([-0.003, 0.039, -0.301])
        parent_dist["left_foot"] = np.array([-0.027, 0.02, -0.138])

        com["hip"] = np.array([0.0, 0.0, 0.0])
        com["left_thigh"] = np.array([0.24,  -0.126,  -0.277])
        com["left_shank"] = np.array([0.264, -0.123,  -0.835])
        com["left_foot"] = np.array([0.242,  -0.119,  -1.051])

        com["right_thigh"] = np.array([-0.236,  -0.126,  -0.277])
        com["right_shank"] = np.array([-0.26,  -0.13,  -0.834])
        com["right_foot"] = np.array([-0.238,  -0.128, -1.044])

        inertia["hip"] = np.diag([0.108509850667, 0.137304730667, 0.0606271146667])

        inertia["right_thigh"] = np.diag([0.0588879339778, 0.0588879339778, 0.00846480799474])
        inertia["right_shank"] = np.diag([0.0229893324138, 0.0229893324138, 0.00297045321452])
        inertia["right_foot"] = np.diag([0.0100261277105, 0.00688495114232, 0.0106987071027])

        inertia["left_thigh"] = np.diag([0.0588879339778, 0.0588879339778, 0.00846480799474])
        inertia["left_shank"] = np.diag([0.0229893324138, 0.0229893324138, 0.00297045321452])
        inertia["left_foot"] = np.diag([0.0100261277105, 0.00688495114232, 0.0106987071027])

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs],
                                                                   inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs],
                                                                  inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])

        hip_id = model.AppendBody(xtrans,
                                  rbdl.Joint.fromJointType("JointTypeFloatingBase"),
                                  hip_body,
                                  "hip"
                                  )

        id_l = hip_id
        id_r = hip_id

        joint_rot_z =  rbdl.Joint.fromJointType("JointTypeRevoluteZ")


        xtrans.E = np.array([[1.0, 0.0, 0.0],
                             [0.0, 0.9945123195648193, 0.10461365431547165],
                             [0.0, -0.10461365431547165, 0.9945123195648193]])

        index = "right_" + "thigh"
        xtrans.r = parent_dist[index]
        id_l = model.AddBody(id_l,
                             xtrans,
                             joint_rot_z,
                             bodies[index],
                             index
                             )

        xtrans.E = np.array([[1.0, 0.0, 0.0],
                             [0.0, 0.9945123195648193, -0.10461365431547165],
                             [0.0, 0.10461365431547165, 0.9945123195648193]])

        index = "left_" + "thigh"
        xtrans.r = parent_dist[index]
        id_l = model.AddBody(id_l,
                             xtrans,
                             joint_rot_z,
                             bodies[index],
                             index
                             )



        xtrans.E = np.eye(3)

        for segs in ["shank", "foot"]:
            index = "right_" + segs
            xtrans.r = parent_dist[index]
            id_r = model.AddBody(id_r,
                                 xtrans,
                                 joint_rot_z,
                                 bodies[index],
                                 index
                                 )

            index = "left_" + segs
            xtrans.r = parent_dist[index]
            id_l = model.AddBody(id_l,
                                 xtrans,
                                 joint_rot_z,
                                 bodies[index],
                                 index
                                 )


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
