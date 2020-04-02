from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
from Controller import PD_Controller
from lib.Python import RMP_runner
import numpy as np
import time
import rospy
from std_msgs.msg import Float32MultiArray

exo = Exoskeleton.Exoskeleton(65,1.57)

client = Client()
client.connect()
h = client.get_obj_handle("hex/Hip")
h.set_pos(0,0,0.0)
h.set_rpy(0.0,0.0,0.0)
q = [0.0]*6
exo = Exoskeleton.Exoskeleton(65,1.57)
pub_vel = rospy.Publisher("vel", Float32MultiArray, queue_size=10)
pub_tau = rospy.Publisher("tau", Float32MultiArray, queue_size=10)
Kp = np.array([550, 100, 550, 550, 100, 550 ])
Kd = np.array([5.0, 1.0, 5.0, 5.0, 1.0, 5.0 ])
controller = PD_Controller.PDController(Kp, Kd)
q_goal = np.asarray([0.05, -0.05, 0.00, 0.05, -0.05, 0.00 ])
qd_goal = np.asarray([ 0, 0, 0, 0, 0, 0 ])
time.sleep(10)
start = time.time()
pos = 0.5
step = 0.1
time_step = 1
while 1:

    q = h.get_all_joint_pos()
    qd = h.get_all_joint_vel()
    e = q_goal - q
    ed = qd_goal - qd
    torque = controller.calc(e, ed)

    for ii, tau in enumerate(torque):
        h.set_joint_effort(ii, tau)

    # vel = Float32MultiArray()
    # tau = Float32MultiArray()
    # vel.data = qd
    # tau.data = torque
    # pub_vel.publish(vel)
    # pub_tau.publish(tau)
    dt = time.time() - start
    if dt > 3:
        h.set_force(0, 0, 5)

    #
    # if dt > 5*time_step:
    #     if pos > 0.4:
    #         pos = pos - step
    #         h.set_pos(0,0,pos)
    #         time_step = time_step + 1
    #     else:
    #         h.set_force(0, 0, 0)


