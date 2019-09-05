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
time.sleep(8)
while 1:
    # Hip
    P = 550.0
    D = 5.0
    q = h.get_joint_pos(0)
    qd = h.get_joint_vel(0)
    msg = Float32()
    msg.data = qd
    pub.publish(msg)
    h.set_joint_effort(0, P*(-0.5-q) + D*(0-qd) )
    # Knee

    # P = 550.0
    # D = 5.0
    # q = h.get_joint_pos(2)
    # qd = h.get_joint_vel(2)
    # msg = Float32()
    # msg.data = qd
    # pub.publish(msg)
    # h.set_joint_effort(2, P*(0.25-q) + D*(0-qd) )
    # # Ankle
    # joint = 1
    # P = 550.0
    # D = 5.0
    # q = h.get_joint_pos(1)
    # qd = h.get_joint_vel(1)
    # msg = Float32()
    # msg.data = qd
    # pub.publish(msg)
    # h.set_joint_effort(1, P*(0.6-q) + D*(0-qd) )




