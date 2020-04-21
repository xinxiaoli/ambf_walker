import sys
from ambf_client import Client
from Controller.PDController import PDController
import numpy as np
import time
from Model.Human import Human
import rospy
import ambf_msgs.msg as ambf

# Create Client and connect
_client = Client()
_client.connect()
print(_client.get_obj_names())

human = Human(_client, 50, 1.5)
q_goal = [0.0] * 7
qd_goal = [0.0] * 7
qdd_goal = [0.0] * 7

gains = ([510, 36.8], [565, 42.89], [354, 45.2], [510, 36.8], [565, 42.89], [354, 45.2])

Kp = np.array([0, gains[0][0], gains[1][0], gains[2][0], gains[3][0], gains[4][0], gains[5][0]])
Kd = np.array([0, gains[0][1], gains[1][1], gains[2][1], gains[3][1], gains[4][1], gains[5][1]])

Controller = PDController(Kp, Kd)


# val = input("Press 'S' to start standing. Press any other button to go to live-interface. You can call 'loop()' later")

# Loop to stay standing
def loop():
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        # Get current states
        q = human.q[:7]
        qd = human.qd[:7]

        # Calc effort from PID
        aq = qdd_goal + Controller.get_tau(q_goal - q, qd_goal - qd)

        # Calc tau from dynamical model
        tau = human.calculate_dynamics(aq)
        human.tau = tau
        rate.sleep()
