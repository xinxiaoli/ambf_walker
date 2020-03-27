"""
This should be moved to a seperate repo later
"""


import abc
import numpy as np
import rbdl
import Model
import time
from threading import Thread
class Exoskeleton(Model.Model):

    def __init__(self, client, mass, height):
        super(Exoskeleton, self).__init__(client, mass, height)
        self._handle = self._client.get_obj_handle('Hip')
        time.sleep(2)
        self.joint_order = ["left_hip", "left_knee", "left_ankle", "right_hip", "right_knee", "right_ankle"]
        self._q = np.asarray([0]*7)
        self._qd = np.asarray([0]*7)
        self._state = (self._q, self._qd)
        self._updater.start()


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

        mass["hip"] = 2.37
        mass["right_thigh"] = 2.11
        mass["left_thigh"] = 2.11
        mass["right_shank"] = 1.2804
        mass["left_shank"] = 1.2804
        mass["right_foot"] = 0.85523
        mass["left_foot"] = 0.85523
        parent_dist = {}

        parent_dist["hip"] = np.array([0.0, 0.0, 0.0])


        parent_dist["right_thigh"] = np.array([-0.237, -0.124, -0.144])
        parent_dist["right_shank"] = np.array([-0.028, 0.06, -0.55])
        parent_dist["right_foot"] = np.array([0.022,0.043,-0.0213])

        parent_dist["left_thigh"] = np.array([0.237, -0.124, -0.144])
        parent_dist["left_shank"] = np.array([-0.028, 0.06, -0.55])
        parent_dist["left_foot"] = np.array([0.022,0.043,-0.0213])

        com["hip"] = np.array([0.0, -0.02, 0.18])
        com["left_thigh"] = np.array([0.02,  0.01,  -0.09])
        com["left_shank"] = np.array([-0.02,  -0.01,  -0.06])
        com["left_foot"] = np.array([-0.08,  -0.06,  0.04])

        com["right_thigh"] = np.array([-0.02,  0.01,  -0.09])
        com["right_shank"] = np.array([0.02,  -0.01,  0.06])
        com["right_foot"] = np.array([0.08,  -0.06, 0.04])

        inertia["hip"] = np.diag([0.0, -0.02, 0.18])

        inertia["left_thigh"] = np.diag([0.0, 0.01, 0.07])
        inertia["left_shank"] = np.diag([0.19, 0.19, 0.0])
        inertia["left_foot"] = np.diag([0.07, 0.07, 0.0])

        inertia["right_thigh"] = np.diag([0.0, 0.01, 0.07])
        inertia["right_shank"] = np.diag([0.19, 0.19, 0.0])
        inertia["right_foot"] = np.diag([0.07, 0.07, 0.0])

        hip_body = rbdl.Body.fromMassComInertia(mass["hip"], com["hip"], inertia["hip"])
        for segs in segments:
            bodies["right_" + segs] = rbdl.Body.fromMassComInertia(mass["right_" + segs], com["right_" + segs], inertia["right_" + segs])
            bodies["left_" + segs] = rbdl.Body.fromMassComInertia(mass["left_" + segs], com["left_" + segs], inertia["left_" + segs])

        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0.0, 0.0, 0.0])
        xtrans.E= np.eye(3)

        self.hip = model.AddBody(0, xtrans, rbdl.Joint.fromJointType("JointTypeFixed"), hip_body,"hip")


        joint_rot_z =  rbdl.Joint.fromJointType("JointTypeRevoluteY")

        xtrans.E =np.array( [ [0, 0, -1.0],
                              [-0.10480714589357376, 0.9944925904273987,  0],
                              [0.9944925904273987, -0.10480714589357376, 0]])

        # xtrans.E = np.array([[1.0, 0.0, 0.0],
        #                      [0.0, 0.9945123195648193, -0.10461365431547165],
        #                      [0.0, 0.10461365431547165, 0.9945123195648193]])

        #xtrans.r = parent_dist["right_thigh"]


        self.right_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["right_thigh"], "right_thigh")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["right_shank"]
        self.right_shank = model.AddBody(self.right_thigh, xtrans, joint_rot_z, bodies["right_shank"], "right_shank")
        xtrans.r = parent_dist["right_foot"]
        self.right_foot = model.AddBody(self.right_shank, xtrans, joint_rot_z, bodies["right_foot"], "right_foot")


        # xtrans.E = np.array([[1.0, 0.0, 0.0],
        #                      [0.0, 0.9945123195648193, -0.10461365431547165],
        #                      [0.0, 0.10461365431547165, 0.9945123195648193]])
        xtrans.E = np.array( [ [0, 0, -1.0], [-0.10480714589357376, 0.9944925904273987,  0], [-0.9944925904273987, -0.10480714589357376, 0]])

        index = "left_" + "thigh"
        xtrans.r = parent_dist["left_thigh"]
        self.left_thigh = model.AddBody(self.hip, xtrans, joint_rot_z, bodies["left_thigh"], "left_thigh")
        xtrans.E = np.eye(3)
        xtrans.r = parent_dist["left_shank"]
        self.left_shank = model.AddBody(self.left_thigh, xtrans, joint_rot_z, bodies["left_shank"], "left_shank")
        xtrans.r = parent_dist["left_foot"]
        self.left_foot = model.AddBody(self.left_shank, xtrans, joint_rot_z, bodies["left_foot"], "left_foot")


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


    def  ambf_to_dyn(self, q):
        return np.asarray( [q[0], q[1], q[3], q[2], q[4], q[6], q[5]])

    def make_foot(self, left_ankle, right_ankle):

        foot = {}
        # TODO put the real nums in

        foot["left_toe"] = {}
        foot["left_heel"] = {}
        foot["right_toe"] = {}
        foot["right_heel"] = {}

        foot["left_toe"]["x"] = left_ankle["x"] + 0.8 * (4.25 / 100.0) * self._height * np.cos(-self._q[3])
        foot["left_toe"]["y"] = left_ankle["y"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["left_toe"]["z"] = left_ankle["z"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["left_heel"]["x"] = left_ankle["x"] - 0.2 * (4.25 / 100.0) * self._height * np.cos(-self._q[3])
        foot["left_heel"]["y"] = left_ankle["y"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["left_heel"]["z"] = left_ankle["z"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[3])
        foot["right_toe"]["x"] = right_ankle["x"] + 0.8 * (4.25 / 100.0) * 1.57 * np.cos(-self._q[6])
        foot["right_toe"]["y"] = right_ankle["y"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])
        foot["right_toe"]["z"] = right_ankle["z"] - 0.05 + 0.8 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])
        foot["right_heel"]["x"] = right_ankle["x"] - 0.2 * (4.25 / 100.0) * self._height * np.cos(-self._q[6])
        foot["right_heel"]["y"] = right_ankle["y"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])
        foot["right_heel"]["z"] = right_ankle["z"] - 0.05 + 0.2 * (4.25 / 100.0) * self._height * np.sin(-self._q[6])

        return foot

    def fk(self):

        fk = {}

        # for index, joint in enumerate(self.joint_order):
        #     point = {grant }
        #     point["x"] = self._model.X_base[2 + index].r[0]
        #     point["y"] = self._model.X_base[2 + index].r[1]
        #     point["z"] = self._model.X_base[2 + index].r[2]
        #     fk[joint] = point
        #
        # feet = self.make_foot(fk["left_ankle"], fk["right_ankle"])

        # data = []
        # for index in xrange(len(self._model.X_base)):
        #     point = []
        #     point.append(self._model.X_base[index].r[0])
        #     point.append(self._model.X_base[index].r[1])
        #     point.append(self._model.X_base[index].r[2])
        #     data.append(point)
        #
        # #
        # # for key, p in feet.iteritems():
        # #     data.append( [ p["x"], p["y"],p["z"] ]  )

        data = []
        point_local = np.array([0.0, 0.0, 0.0])

        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_thigh, point_local))
        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_shank, point_local))
        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.left_foot, point_local))

        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.hip, point_local))
        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_thigh, point_local))
        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_shank, point_local))
        data.append(rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.right_foot, point_local))

        fk = data
        return fk


    def calculate_dynamics(self, q_d, qd_d, qdd_d):
        q = self.ambf_to_dyn(q_d)
        qd = self.ambf_to_dyn(qd_d)
        qdd = self.ambf_to_dyn(qdd_d)
        tau = np.asarray([0.0] * 7)
        rbdl.InverseDynamics(self._model, q, qd, qdd, tau)
        return self.ambf_to_dyn(tau)
