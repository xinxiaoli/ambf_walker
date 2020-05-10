#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints


class Initialize(smach.State):

    def __init__(self, model, outcomes=['Initializing', 'Initialized']):

        smach.State.__init__(self, outcomes=outcomes)
        self._model = model
        self.rate = rospy.Rate(1000)
        tf = 2.0
        dt = 0.01
        self.hip, self.knee, self.ankle = self._model.stance_trajectory(tf=tf, dt=dt)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)

        self.total = tf / dt
        self.count = 0

    def execute(self, userdata):

        self._model.handle.set_rpy(0.25, 0, 0)
        self._model.handle.set_pos(0, 0, 2.0)

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
        self.rate = rospy.Rate(10)
        self._model = model

    def execute(self, userdata):
        # Your state execution goes here
        z = self._model.handle.get_pos().z
        print z
        if z < -0.23:
            self._model.handle.set_force(0.0, 0.0, 0.0)
            self.rate.sleep()
            return 'Stabilized'
        else:
            height = z - 0.0001
            self._model.handle.set_pos(0, 0, height)
            self.rate.sleep()
            return 'Stabilizing'



class GMRTest(smach.State):

    def __init__(self, model, outcomes=["Following", "Followed"]):
        smach.State.__init__(self, outcomes=outcomes)
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self._model = model
        self.count = 0
        self.runner = TPGMMRunner.TPGMMRunner("TMGMM" + ".pickle")

    def execute(self, userdata):
        # Your state execution goes here

        if self.count < 10:

            q = self._model.q
            qd = self._model.qd
            qdd = self._model.qdd
            self.runner.step(q[0], q[0])
            xdd = self.runner.xdd
            self.msg.controllers = ["LQR", "PD", "PD", "PD", "PD", "PD", "PD"]
            qdd[0] = xdd[0]

            self.count += 1
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.rate.sleep()
            self.pub.publish(self.msg)
            return "Initializing"
        else:
            return "Initialized"

