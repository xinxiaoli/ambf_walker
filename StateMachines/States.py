#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
from GaitAnaylsisToolkit.LearningTools.Runner import TPGMMRunner
from std_msgs.msg import Float32MultiArray
from Model import Model
from std_msgs.msg import Empty,String
from Controller import MPController
from ambf_walker.srv import DesiredJointsCmdRequest, DesiredJointsCmd


class Initialize(smach.State):

    def __init__(self, model, outcomes=['Initializing', 'Initialized']):

        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')

        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
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

        self._model.handle.set_rpy(0.25, 0, 0)
        self._model.handle.set_pos(0, 0, 1.0)

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
            self.msg.controller = "Dyn"
            self.pub.publish(self.msg)
            #self.send(q, qd, qdd, "Dyn", [])
            self.rate.sleep()

            return 'Initializing'
        else:
            return "Initialized"


class Main(smach.State):

    def __init__(self, model,outcomes=["Poly"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.Subscriber("Mode", String, callback=self.mode_cb)
        rospy.wait_for_service('joint_cmd')

        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.have_msg = False
        self.msg = String
        self.Rate = rospy.Rate(100)

    def mode_cb(self, msg):

        if not self.have_msg:
            self.msg = msg
            self.have_msg = True

    def execute(self, userdata):

        rate = rospy.Rate(1000)
        self.have_msg = False
        while not self.have_msg:
            rate.sleep()

        return self.msg.data

class DMP(smach.State):

    def __init__(self, model,outcomes=["stepping", "stepped"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runner = self._model.get_runner()
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        count = self.count

        if count == 0:
            start = []
            for q in self._model.q[0:6]:
                start.append(np.array([q]))
            print(start)
            print(self.runner.x)
            self.runner.update_start(start)

        if count < self.runner.get_length():

            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controller = "Dyn"
            self.pub.publish(self.msg)
            #self.send(q, qd, qdd,"Dyn",[])
            self.count += 1
            self.rate.sleep()
            return "stepping"
        else:
            self.count = 0
            return "stepped"


class GoTo(smach.State):

    def __init__(self, model, outcomes=["Sending", "Waiting"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.Subscriber("Traj", DesiredJoints, callback=self.traj_cb)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.have_msg = False
        self.Rate = rospy.Rate(100)
        self.q = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)


    def traj_cb(self, msg):
        self.q = DesiredJoints()
        if not self.have_msg:
            self.q = msg
            self.have_msg = True

    def execute(self, userdata):
        # Your state execution goes here
        self.Rate.sleep()
        if self.have_msg:
            q_d = np.array(list(self.q.q) + [0.0])
            qd_d = np.array(list(self.q.qd) + [0.0])
            qdd_d = np.array(list(self.q.qdd) + [0.0])
            msg = DesiredJoints()
            msg.q = q_d
            msg.qd = qd_d
            msg.qdd = qdd_d
            msg.controller = "Dyn"
            self.pub.publish(self.msg)

            self.have_msg = False
            return "Sending"
        else:
            return "Waiting"


class Listening(smach.State):

    def __init__(self, model, outcomes=["Sending", "Waiting"]):
        smach.State.__init__(self, outcomes=outcomes, output_keys=['q'])


        rospy.Subscriber("Traj", DesiredJoints, callback=self.traj_cb)
        self._model = model
        self.have_msg = False
        self.Rate = rospy.Rate(100)
        self.q = []

    def traj_cb(self, msg):
        self.q = []
        if not self.have_msg:
            current_joints = self._model.q
            for q, q_d in zip(tuple(current_joints), msg.q):
                self.q.append(Model.get_traj(q, q_d, 0.0, 0.0, 1.0, 0.01))
            self.have_msg = True

    def execute(self, userdata):
        # Your state execution goes here
        userdata.count = 0

        self.Rate.sleep()
        if self.have_msg:
            userdata.q = self.q
            self.have_msg = False
            return "Sending"
        else:
            return "Waiting"

class Follow(smach.State):

    def __init__(self, model, outcomes=['Following', 'Followed']):

        smach.State.__init__(self, outcomes=outcomes,
                              input_keys=['q'],
                              output_keys=['q'])

        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(100)

        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        q = userdata.q
        msg = DesiredJoints()
        count = self.count
        if count <= len(q[0]["q"]) - 1:

            q_d = np.array([q[0]["q"][count].item(), q[1]["q"][count].item(),
                          q[2]["q"][count].item(), q[3]["q"][count].item(),
                          q[4]["q"][count].item(), q[5]["q"][count].item(), 0.0])

            qd_d = np.array([q[0]["qd"][count].item(), q[1]["qd"][count].item(),
                          q[2]["qd"][count].item(), q[3]["qd"][count].item(),
                          q[4]["qd"][count].item(), q[5]["qd"][count].item(), 0.0])

            qdd_d = np.array([q[0]["qdd"][count].item(), q[1]["qdd"][count].item(),
                          q[2]["qdd"][count].item(), q[3]["qdd"][count].item(),
                          q[4]["qdd"][count].item(), q[5]["qdd"][count].item(), 0.0])

            msg.q = q_d
            msg.qd = qd_d
            msg.qdd = qdd_d
            msg.controller = "Dyn"
            #self.send(q_d, qd_d, qdd_d,"Dyn", [])
            self.pub.publish(msg)
            self.count += 1
            self.rate.sleep()
            return "Following"
        else:
            self.count = 0
            return "Followed"


class LowerBody(smach.State):

    def __init__(self, model, outcomes=['Lowering', 'Lowered']):

        smach.State.__init__(self, outcomes=outcomes)
        self._model = model
        self.rate = rospy.Rate(1)
        self.step = 0.00000000001
        self.final_height = -0.38

    def execute(self, userdata):

        self.rate.sleep()
        current = self._model.handle.get_pos().z

        if current > self.final_height:
            self._model.handle.set_pos(0.0,0.0, current-self.step)
            self._model.handle.set_rpy(0.25, 0, 0)
            return 'Lowering'
        else:
            self._model.handle.set_rpy(0.25, 0, 0)
            self._model.handle.set_force(0.0, 0.0, 0.0)
            return "Lowered"


class MPC(smach.State):

    def __init__(self, model, outcomes=["MPCing", "MPCed"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runner = model.get_runner()
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        msg = DesiredJoints()
        msg.controller = "MPC"

        if self.count < self.runner.get_length():

            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            msg.qdd = qdd #[self.count]
            self.send(q, qd, qdd, "MPC", [self.count])
            #self.pub.publish(msg)
            self.rate.sleep()
            self.count += 1
            return "MPCing"
        else:
            return "MPCed"


class LQR(smach.State):

    def __init__(self, model, outcomes=["LQRing", "LQRed"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runner = model.get_runner()
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):


        if self.count < self.runner.get_length()-1:

            self.runner.step()
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            msg = DesiredJoints()
            # msg.q = q
            # msg.qd = qd
            # msg.qdd = qdd
            # msg.controller = "LQR"
            # msg.other = [self.count]
            # self.pub.publish(msg)

            self.send(q, qd, qdd, "LQR", [self.count])
            self.rate.sleep()
            self.count += 1
            return "LQRing"
        else:
            # while 1:
            #     q = [-0.7, 0.5, -0.2, -0.7, 0.5, -0.2, 0.0]
            #     qd = [0.0]*7
            #     qdd = [0.0]*7
            #     self.send(q, qd, qdd, "Dyn", [self.count])
            return "LQRed"
