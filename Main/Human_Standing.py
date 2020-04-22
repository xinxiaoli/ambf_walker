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
left_order = human.ambf_order_crutch_left
right_order = human.ambf_order_crutch_right
joints = human.handle.get_joint_names()

print("Setting pos to 0 for all")
# body = human.handle
# body.set_pos(0,0,0)
# children = body.get_children_names()
#
# for child in children:
#     _client.get_obj_handle(child).set_pos(0,0,0)

num_joints = len(joints)
q_goal = np.array([0.0] * num_joints)
qd_goal = np.array([0.0] * num_joints)
qdd_goal = np.array([0.0] * num_joints)

Controller = PDController(0,0)

K_hip = [10, 0]
K_knee = [10, 0]
K_ankle = [10, 0]


def init_k_vals():
    Kp = np.zeros(num_joints)
    Kp[left_order['hip']] = K_hip[0]
    Kp[left_order['knee']] = K_knee[0]
    Kp[left_order['ankle']] = K_ankle[0]
    Kp[right_order['hip']] = K_hip[0]
    Kp[right_order['knee']] = K_knee[0]
    Kp[right_order['ankle']] = K_ankle[0]

    Kd = np.zeros(num_joints)
    Kd[left_order['hip']] = K_hip[1]
    Kd[left_order['knee']] = K_knee[1]
    Kd[left_order['ankle']] = K_ankle[1]
    Kd[right_order['hip']] = K_hip[1]
    Kd[right_order['knee']] = K_knee[1]
    Kd[right_order['ankle']] = K_ankle[1]

    Kp = np.diag(Kp)
    Kd = np.diag(Kd)

    Controller = PDController(Kp, Kd)
    return Controller


# Loop to stay standing
def loop():
    # rate = rospy.Rate(100)
    start = rospy.get_time()
    now = rospy.get_time()
    while abs(now - start) < 5:
        now = rospy.get_time()
        # Get current states
        q = human.q
        qd = human.qd
        print("Q: ")
        print(q)
        # Calc effort from PID
        aq = qdd_goal + Controller.get_tau(q_goal - q, qd_goal - qd)
        print("Aq:")
        print(aq)

        # Calc tau from dynamical model
        tau = human.calculate_dynamics(aq)
        human.update_torque(tau)
        # rate.sleep()
    print("5 seconds over")

Controller = init_k_vals()
# loop()
