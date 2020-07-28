#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints

from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
from std_msgs.msg import Float64

class Initialize(smach.State):

    def __init__(self, model, outcomes=['Initializing', 'Initialized']):

        smach.State.__init__(self, outcomes=outcomes)
        self._model = model
        self.rate = rospy.Rate(100)
        tf = 2.0
        dt = 0.01
        self.hip, self.knee, self.ankle = self._model.stance_trajectory(tf=tf, dt=dt)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)

        self.total = tf / dt
        self.count = 0

    def execute(self, userdata):

        # self._model.handle.set_rpy(0.25, 0, 0)
        # self._model.handle.set_pos(0, 0, 2.0)

        if self.count <= self.total - 1:

            q = np.array([self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(),
                          self.hip["q"][self.count].item(), self.knee["q"][self.count].item(),
                          self.ankle["q"][self.count].item(), 0.0])

            qd = np.array([self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(),
                           self.hip["qd"][self.count].item(), self.knee["qd"][self.count].item(),
                           self.ankle["qd"][self.count].item(), 0.0])

            qdd = np.array([self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(),
                            self.hip["qdd"][self.count].item(), self.knee["qdd"][self.count].item(),
                            self.ankle["qdd"][self.count].item(), 0.0])

            self.count += 1
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controllers = ["PD", "PD", "PD", "PD", "PD", "PD", "PD"]
            self.rate.sleep()
            self.pub.publish(self.msg)
            return "Initializing"
        else:
            return "Initialized"


class Stabilize(smach.State):
    def __init__(self, model, outcomes=['Stabilizing', 'Stabilized']):
        smach.State.__init__(self, outcomes=outcomes)
        self.rate = rospy.Rate(100)
        self._model = model

    def execute(self, userdata):
        # Your state execution goes here
        z = self._model.handle.get_pos().z
        if z < 1.5:
            self._model.handle.set_force(0.0, 0.0, 0.0)
            self.rate.sleep()
            return 'Stabilized'
        else:
            height = 2.0
            self._model.handle.set_pos(0, 0, height)
            self.rate.sleep()
            return 'Stabilizing'



class GMRTest(smach.State):

    def __init__(self, model, outcomes=["Following", "Followed"]):
        smach.State.__init__(self, outcomes=outcomes)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        # self.point = rospy.Publisher("set", Float64, queue_size=1)
        self.lqr = rospy.Publisher("LQR", DesiredJoints, queue_size=1)
        self.msg = DesiredJoints()
        self.msg_control = DesiredJoints()
        self._model = model
        self.count = 0
#        self.runner = TPGMMRunner.TPGMMRunner("/home/nathaniel/catkin_ws/src/ambf_walker/config/poly" + ".pickle")

    def execute(self, userdata):
        # Your state execution goes here

        if self.count < self.runner.get_length():
            # self.msg.controllers = ["PD", "PD", "PD", "LQR", "PD", "PD", "PD"]
            # self.msg = DesiredJoints()
            # self.msg_control = DesiredJoints()
            # length = len(self._model.q)
            # q = self._model.q
            # qd = self._model.qd
            # qdd = np.zeros(length)
            # x = self.runner.step(q[3],qd[3])
            # dx = self.runner.dx[0]
            # ddx = self.runner.ddx[0]
            # self.msg.q = q
            # self.msg.q[3] = x
            # self.msg.qd = qd
            # self.msg.qdd = qdd
            # self.msg.qdd[3] = ddx
            # self.count += 1
            # self.lqr.publish(self.msg)
            # self.pub.publish(self.msg)
            self.rate.sleep()
            return "Following"
        else:
            self.pub.publish(self.msg)
            self.lqr.publish(self.msg)
            return "Followed"

