import time
from Tkinter import *

import numpy as np
import rospy
from ambf_client import Client
from std_msgs.msg import Float32MultiArray,Float32
from Controller import PD_Controller
from Model import Exoskeleton
from Utlities import Plotter


from ambf_client import Client

client = Client()
client.connect()
pub = rospy.Publisher("Vel", Float32, queue_size=1)
h = client.get_obj_handle("hex/Hip")
h.set_pos(0,0,0.5)
h.set_rpy(0,0,0)

exo = Exoskeleton.Exoskeleton(65,1.57)
#plotter = Plotter.Plotter(exo)
Kp = np.array([0, 20.0, 200, 200, 525, 525, 525 ])
Kd = np.array([0, 0.0, 0, 0.0, 5.0, 5.0, 5.0 ])
controller = PD_Controller.PDController(Kp, Kd)
q_goal = np.array([ 0, -0.5, 0.25, 0.6, -0.5, 0.25, 0.6])
qd_goal = np.array([ 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
time.sleep(4)
while 1:
    q = h.get_all_joint_pos()
    qd = h.get_all_joint_vel()
    q.insert(0, 0.0)
    qd.insert(0, 0.0)
    exo.update_joints(np.asarray(q), np.asarray(qd))
    e = q_goal - q
    ed = qd_goal - qd
    aq = controller.calc(e, ed)
    tau = exo.calculate_dynamics(q_goal, qd_goal, aq)
    h.set_joint_effort(0, tau[1])
    # h.set_joint_effort(1, tau[2])
    # h.set_joint_effort(2, tau[3])
    #plotter.update()





