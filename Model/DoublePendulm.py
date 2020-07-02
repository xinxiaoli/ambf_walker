
import abc
import numpy as np
import rbdl
import time
import rospy
from threading import Thread
from lib.GaitCore.Bio import Joint, Leg
from lib.GaitCore.Core import Point
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from Utlities import PendPlotter
from . import Model

class DoublePendulum(Model.Model):

    def __init__(self, client):
        super(DoublePendulum, self).__init__(client, 0, 0)
        self._handle = self._client.get_obj_handle('Sphere')
        self.q = 2 * [0.0]
        self.qd = 2 * [0.0]
        time.sleep(2)
        self._state = (self._q, self._qd)
        #self.plt = PendPlotter.Plotter(self)
        self._updater.start()


    def torque_cb(self, tau):
        self.update_torque(list(tau.effort))

    def update_torque(self, tau):
        """

        :type tau: JointState
        """
        self.tau = tau
        self._enable_control = True

    @property
    def enable_control(self):
        return self._enable_control

    @enable_control.setter
    def enable_control(self, value):
        self._enable_control = value

    @property
    def handle(self):
        return self._handle

    @handle.setter
    def handle(self, value):
        self._handle = value

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

    @abc.abstractmethod
    def dynamic_model(self, mass, height):
        m1 = 1
        m2 = 1
        l1 = 0.5
        l2 = 0.5
        r1 = 0
        r2 = -0.6
        I1 = np.diag([0.06, 0.06, 0.002])
        I2 = np.diag([0.06, 0.06, 0.002])
        joint_rot_y = rbdl.Joint.fromJointType("JointTypeRevoluteY")

        xtrans_1 = rbdl.SpatialTransform()
        xtrans_1.r = np.array([0.0, 0.0, 0.0])

        xtrans_2 = rbdl.SpatialTransform()
        xtrans_2.r = np.array([0.0, 0.0, r2])

        model = rbdl.Model()
        link0 = rbdl.Body.fromMassComInertia(0,
                                             np.array([0., 0, 0.]),
                                             np.diag([0.0, 0.0, 0.0])
                                             )

        body1 = rbdl.Body.fromMassComInertia(m1, np.array([0.0, 0.0, -0.35]), I1)

        body2 = rbdl.Body.fromMassComInertia(m2, np.array([0.0, 0.0, -0.35]), I2)
        self.b0 = model.AppendBody(rbdl.SpatialTransform(), joint_rot_y, link0)
        self.b1 = model.AppendBody(xtrans_2, joint_rot_y, body1)
        self.b2 = model.AppendBody(xtrans_2, joint_rot_y, body2)
        model.gravity = np.array([0, 0, -9.81])
        return model

    def update(self):
        """

        :return:
        """
        rate = rospy.Rate(1000)  # 1000hz
        q_msg = Float32MultiArray()
        while 1:
            self.q = self.handle.get_all_joint_pos()
            self.qd = self.handle.get_all_joint_vel()
            self._joint_num = self.q.size
            q_msg.data = self.q
            self.q_pub.publish(q_msg)
            if self._enable_control:
                self.handle.set_all_joint_effort(self.tau)

            rate.sleep()
            #self.plt.update()

    @abc.abstractmethod
    def  ambf_to_dyn(self, q):
        pass

    @abc.abstractmethod
    def fk(self):
        """

        :param model: model
        :type model: rbdl.model
        :return: 2D array of the joint locations
        """

        x = []
        y = []

        point_local = np.array([0.0, 0.0, 0.0])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b0, point_local)
        x.append(data[0])
        y.append(data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b1, point_local)
        x.append(data[0])
        y.append(data[2])
        data = rbdl.CalcBodyToBaseCoordinates(self._model, self.q, self.b2, point_local)
        x.append(data[0])
        y.append(data[2])
        return (x, y)

    @abc.abstractmethod
    def update_state(self, q, qd):
        self.state = q + qd

    def calculate_dynamics(self, qdd):
        tau = np.asarray([0.0] * self._joint_num)
        rbdl.InverseDynamics(self._model, self.q[0:2], self.qd[0:2], qdd[0:2], tau)
        return tau

