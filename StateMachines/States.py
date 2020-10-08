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
import matplotlib.pyplot as plt
from ambf_walker.srv import DesiredJointsCmdRequest, DesiredJointsCmd
from ilqr.controller import RecedingHorizonController
from ilqr.cost import PathQsRCost
from ilqr import iLQR
from ilqr.dynamics import FiniteDiffDynamics
from GaitAnaylsisToolkit.LearningTools.Runner import GMMRunner
import numpy.polynomial.polynomial as poly


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


class MPC2(smach.State):

    def __init__(self, model, outcomes=["MPC2ing", "MPC2ed"]):
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
            x = self.runner.x
            dx = self.runner.dx
            ddx = self.runner.ddx
            q = np.append(x, [0.0])
            qd = np.append(dx, [0.0])
            qdd = np.append(ddx, [0.0])
            self.send(q, qd, qdd, "MPC", [self.count])
            return "MPC2ing"
        else:
            return "MPC2ed"

            pass

class LQR(smach.State):

    def __init__(self, model, outcomes=["LQRing", "LQRed"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.rate = rospy.Rate(100)
        file = "/home/nathanielgoldfarb/linearize_model/test.npy"
        with open(file, 'rb') as f:
            self.us2 = np.load(f)
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0

    def execute(self, userdata):

        if self.count < 200:
            q = np.array(7*[0.0])
            qd = np.array(7*[0.0])
            qdd = np.append(self.us2[self.count], [0.0])
            self.send(q, qd, qdd, "Temp", [self.count])
            self.rate.sleep()
            self.count += 1
            return "LQRing"
        else:
            return "LQRed"


class Temp(smach.State):

    def __init__(self, model, outcomes=["Temping", "Temped"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self.runner = TPGMMRunner.TPGMMRunner("/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/Train/gotozero.pickle")
        self._model = model
        self.runner = model.get_runner()
        self.rate = rospy.Rate(1000)
        #self.setup()

    def setup(self):

        J_hist = []

        def on_iteration(iteration_count, xs, us, J_opt, accepted, converged):
            J_hist.append(J_opt)
            info = "converged" if converged else ("accepted" if accepted else "failed")
            print("iteration", iteration_count, info, J_opt)

        max_bounds = 8.0
        min_bounds = -8.0
        def f(x, u, i):
            diff = (max_bounds - min_bounds) / 2.0
            mean = (max_bounds + min_bounds) / 2.0
            u = diff * np.tanh(u) + mean
            y = Model.runge_integrator(self._model.get_rbdl_model(), x, 0.01, u)
            return np.array(y)

        dynamics = FiniteDiffDynamics(f, 12, 6)

        x_path = []
        u_path = []
        count = 0
        N = self.runner.get_length()
        while count < self.runner.get_length():
            count += 1
            self.runner.step()
            u_path.append(self.runner.ddx.flatten().tolist())
            x = self.runner.x.flatten().tolist() + self.runner.dx.flatten().tolist()
            x_path.append(x)

        u_path = u_path[:-1]
        expSigma = self.runner.get_expSigma()
        size = expSigma[0].shape[0]
        Q = [np.zeros((size * 2, size * 2))] * len(expSigma)
        for ii in range(len(expSigma) - 2, -1, -1):
            Q[ii][:size, :size] = np.linalg.pinv(expSigma[ii])

        x0 = x_path[0]
        x_path = np.array(x_path)
        self.u_path = np.array(u_path)
        R = 0.1 * np.eye(dynamics.action_size)
        #
        cost2 = PathQsRCost(Q, R, x_path=x_path, u_path=self.u_path)
        #
        # # Random initial action path.
        ilqr2 = iLQR(dynamics, cost2, N - 1)

        self.cntrl = RecedingHorizonController(x0, ilqr2)

    def execute(self, userdata):
        count = 0
        for xs2, us2 in self.cntrl.control(self.u_path):
            q = np.array([0.0]*7)
            qd = np.array([0.0]*7)
            qdd = np.append(us2, [0.0])
            print(qdd)
            self.send(q, qd, qdd, "Temp", [count])
            self.rate.sleep()
            count += 1
            print(count)



class StairDMP(smach.State):

    def __init__(self, model,outcomes=["stairing", "staired"]):
        smach.State.__init__(self, outcomes=outcomes)
        rospy.wait_for_service('joint_cmd')
        self.send = rospy.ServiceProxy('joint_cmd', DesiredJointsCmd)
        self._model = model
        self.runnerZ = GMMRunner.GMMRunner("/home/nathanielgoldfarb/stair_traj_learning/Main/toeZ_all.pickle")  # make_toeZ([file1, file2], hills3, nb_states, "toe_IK")
        self.runnerY = GMMRunner.GMMRunner("/home/nathanielgoldfarb/stair_traj_learning/Main/toeY_all.pickle")  # make_toeY([file1, file2], hills3, nb_states, "toe_IK")
        self.rate = rospy.Rate(100)
        self.msg = DesiredJoints()
        self.pub = rospy.Publisher("set_points", DesiredJoints, queue_size=1)
        self.count = 0
        self.init_joint_angles = []

    def smooth_curve(self, data, t, order):

        coefs = poly.polyfit(t, data, order)
        ffit = poly.Polynomial(coefs)  # instead of np.poly1d
        return ffit(t)

    def execute(self, userdata):

        if self.count == 0:
            self.init_joint_angles = self._model.q

            self.runnerZ.update_start(0)
            self.runnerZ.update_goal(192)

            self.runnerY.update_start(-225.0)
            self.runnerY.update_goal(258.0)


            self.hip_angles = []
            self.knee_angles = []
            self.ankle_angles = []

            pathZ = self.runnerZ.run()
            pathY = self.runnerY.run()

            for y, x in zip(pathZ, pathY):
                joint_angle = self._model.leg_inverse_kinimatics([y, x], hip_location=[-483.4, 960.67])
                self.hip_angles.append(joint_angle[0][0])
                self.knee_angles.append(joint_angle[1][0])
                self.ankle_angles.append(joint_angle[2][0])

            t = np.linspace(0, 100, len(self.knee_angles))

            self.hip_angles = self.smooth_curve(self.hip_angles, t, 6)
            self.knee_angles = self.smooth_curve(self.knee_angles, t, 6)
            self.ankle_angles = self.smooth_curve(self.ankle_angles, t, 6)

            self.hip_vel = []
            self.knee_vel = []
            self.ankle_vel = []

            tf = len(self.hip_angles)
            V_hip = 2.0
            V_knee = 1.0
            V_ankle = 0.5
            alpha = 5.0
            for t in range(tf):

                if  0 <= t and  t <= int(tf/alpha):
                    self.hip_vel.append((alpha*V_hip*t)/tf)
                    self.knee_vel.append((alpha*V_knee*t)/tf)
                    self.ankle_vel.append((alpha*V_knee*t)/tf)

                if int(tf/alpha) < t and  t <= int((alpha*tf - tf )/alpha):
                    self.hip_vel.append(V_hip)
                    self.knee_vel.append(V_knee)
                    self.ankle_vel.append(V_knee)

                if int((alpha*tf - tf )/alpha ) < t and t < tf:
                    self.hip_vel.append((-alpha * V_hip * t) / tf + alpha*V_hip)
                    self.knee_vel.append((-alpha * V_knee * t) / tf + alpha*V_knee)
                    self.ankle_vel.append((-alpha * V_knee * t) / tf + alpha*V_ankle)
            #
            self.hip_vel.append(0.0)
            self.knee_vel.append(0.0)
            self.ankle_vel.append(0.0)
            plt.plot(self.knee_vel)
            plt.show()

        if self.count < self.runnerY.get_length()-2:

            # self.runnerZ.step()
            # self.runnerY.step()
            # x = self.runnerZ.x[0].item()
            # dx = self.runnerZ.dx
            # ddx = self.runnerZ.ddx
            #
            # y = self.runnerY.x[0].item()
            # dt = self.runnerY.dx
            # ddy = self.runnerY.ddx
            # x = self.pathZ[self.count][0]
            # y = self.pathY[self.count][0]
            # joint_angle = self._model.leg_inverse_kinimatics([y, x], hip_location=[-483.4, 960.67])

            q = np.array([self.init_joint_angles[0],
                          self.init_joint_angles[1],
                          self.init_joint_angles[2],
                          self.hip_angles[self.count],
                          self.knee_angles[self.count],
                          self.ankle_angles[self.count],
                          0.0])

            qd = np.array([self.init_joint_angles[0],
                           self.init_joint_angles[1],
                           self.init_joint_angles[2],
                           self.hip_vel[self.count],
                           self.knee_vel[self.count],
                           self.ankle_vel[self.count],
                           0.0])

            # qdd = np.array([0.0,
            #                 0.0,
            #                 0.0,
            #                 self.hip_acl[self.count],
            #                 self.knee_acl[self.count],
            #                 self.ankle_acl[self.count],
            #                 0.0])

            # qd = np.array([0.0]*7)
            qdd = np.array([0.0]*7)
            # q[3] = 0# 1.50971 - 0.5*3.14
            # q[4] =  -q[4] # 0# 0.523599
            # q[5] = 0.0
            #self._model.handle.set_multiple_joint_pos(q, [0,1,2,3,4,5,6])
            self.count += 1
            self.msg.q = q
            self.msg.qd = qd
            self.msg.qdd = qdd
            self.msg.controller = "Dyn"
            self.pub.publish(self.msg)
            #self.send(q, qd, qdd, "Dyn", [])
            self.count += 1
            self.rate.sleep()
            return "stairing"
        else:
            self.count = 0

            #plt.plot(self.Zpoints)
            plt.show()
            return "staired"