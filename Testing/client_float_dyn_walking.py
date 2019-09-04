from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
from lib.Python import RMP_runner
import numpy as np
from Controller import PD_Controller
import time
import rospy
from std_msgs.msg import Float32MultiArray
from Tkinter import *

exo = Exoskeleton.Exoskeleton(65,1.57)
plotter = Plotter.Plotter(exo)
client = Client()
client.connect()
h = client.get_obj_handle("hex/Hip")
#h.set_force(0, 0, 100.0)
#h.set_rpy(0.0,0.0,0.0)
q = [0.0]*6

Lankle_runner = RMP_runner.RMP_runner("../config/ankle_left.xml")
Lhip_runner = RMP_runner.RMP_runner("../config/hip_left.xml")
Lknee_runner = RMP_runner.RMP_runner("../config/knee_left.xml")

Rhip_runner = RMP_runner.RMP_runner("../config/hip_right.xml")
Rknee_runner = RMP_runner.RMP_runner("../config/knee_right.xml")
Rankle_runner = RMP_runner.RMP_runner("../config/ankle_right.xml")

q_goal = np.asarray([0.0]*7)
qd_goal = np.asarray([0.0]*7)
qdd_goal = np.asarray([0.0]*7)

gains = ( [ 0.10, 0.00], [.00, 00], [0.0, 0.00],
          [ 0.10, 0.00], [.00, 0.0], [0.0, 0.00])

Kp = np.array([0,gains[0][0],gains[1][0],gains[2][0],gains[3][0],gains[4][0],gains[5][0] ])
Kd = np.array([0,gains[0][1],gains[1][1],gains[2][1],gains[3][1],gains[4][1],gains[5][1] ])
Controller = PD_Controller.PDController(Kp, Kd)

last_time = time.time()
force = 102.0
h.set_pos(0,0,2.2)
h.set_rpy(0,0,0)
flag = False
time.sleep(5)
print h.get_joint_names()
my_pub = rospy.Publisher("torque", Float32MultiArray, queue_size=10)

while 1:


    # q_goal[1] , qd_goal[1] , qdd_goal[1] = Lhip_runner.step(1.0)
    # q_goal[3] , qd_goal[3] , qdd_goal[3]  = Lknee_runner.step(1.0)
    # q_goal[2] , qd_goal[2] , qdd_goal[1]  = Lankle_runner.step(1.0)
    # q_goal[4] , qd_goal[4] , qdd_goal[4]  = Rhip_runner.step(1.0)
    # q_goal[6] , qd_goal[6] , qdd_goal[6]  = Rknee_runner.step(1.0)
    # q_goal[5] , qd_goal[5] , qdd_goal[5]  = Rankle_runner.step(1.0)

    msg = Float32MultiArray()
    q_goal = np.asarray([0.0] * 7)
    qd_goal = np.asarray([0.0] * 7)
    qdd_goal = np.asarray([0.0] * 7)
    q = h.get_all_joint_pos()
    qd = h.get_all_joint_vel()
    q.insert(0, 0.0)
    qd.insert(0, 0.0)
    torque = np.multiply(Kp, q_goal - q)
    #torque = Controller.calc(q_goal - q, qd_goal - qd)
    #torque = exo.calculate_dynamics(q, q, aq)
    for ii, tau in enumerate(torque[1:]):
        h.set_joint_effort(ii, tau)
        msg.data.append(tau)

    my_pub.publish(msg)
    exo.update_joints(np.asarray(q), np.asarray([0.0]*7))
    plotter.update()


