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
rospy.sleep(1)

human = Human(_client, 1.0, 1.5)
left_order = human.ambf_order_crutch_left
right_order = human.ambf_order_crutch_right
joints = human.handle.get_joint_names()
print(joints)

# print("Setting pos to 0 for all")
body = human.handle

def set_body():
    body.set_pos(0,0,.2)
    body.set_rpy(1.3,0,0)


children = body.get_children_names()

num_joints = len(joints)
q_goal = np.array([0.0] * num_joints)
qd_goal = np.array([0.0] * num_joints)
qdd_goal = np.array([0.0] * num_joints)

Controller = PDController(0,0)

k_hip = [30, 1]
k_knee = [30, 1]
k_ankle = [20, 1]


def init_k_vals():
    kp = np.zeros(num_joints)
    kp[left_order['hip']] = k_hip[0]
    kp[left_order['knee']] = k_knee[0]
    kp[left_order['ankle']] = k_ankle[0]
    kp[right_order['hip']] = k_hip[0]
    kp[right_order['knee']] = k_knee[0]
    kp[right_order['ankle']] = k_ankle[0]

    kd = np.zeros(num_joints)
    kd[left_order['hip']] = k_hip[1]
    kd[left_order['knee']] = k_knee[1]
    kd[left_order['ankle']] = k_ankle[1]
    kd[right_order['hip']] = k_hip[1]
    kd[right_order['knee']] = k_knee[1]
    kd[right_order['ankle']] = k_ankle[1]

    kp = np.diag(kp)
    kd = np.diag(kd)

    Controller = PDController(kp, kd)
    return Controller


# Loop to stay standing
def loop():
    Controller = init_k_vals()
    # rate = rospy.Rate(100)
    start = rospy.get_time()
    now = rospy.get_time()
    while abs(now - start) < 5:
        now = rospy.get_time()
        # Get current states
        q = human.q
        qd = human.qd
        # print("Q: ")
        # print(q)
        # Calc effort from PID
        aq = qdd_goal + Controller.get_tau(q_goal - q, qd_goal - qd)
        # print("Aq:")
        # print(aq)

        # Calc tau from dynamical model
        tau = human.calculate_dynamics(aq)
        human.update_torque(tau)
        # rate.sleep()
    print("5 seconds over")

def loop_free():
    print("removing controller oh boy")
    body._cmd.enable_position_controller = False
    loop()


# loop()

