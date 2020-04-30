import sys
from ambf_client import Client
from Controller.PDController import PDController
from Controller.TrajectoryGen import TrajectoryGen
import numpy as np
import time
from Model.Human import Human
import rospy
from std_msgs.msg import Float32, Float32MultiArray
import ambf_msgs.msg as ambf

# Create Client and connect
_client = Client()
_client.connect()
rospy.sleep(1)

human = Human(_client, 1.0, 1.5)
body = human.handle

leg_segs = ['hip', 'knee', 'ankle']
left_order = human.ambf_order_crutch_left
right_order = human.ambf_order_crutch_right

children = body.get_children_names()
print(children)

# Targets joint values
num_joints = len(children)

# controller inits
Controller = PDController(0, 0)
hip_traj = TrajectoryGen()
knee_traj = TrajectoryGen()
ankle_traj = TrajectoryGen()
trajs = [hip_traj, knee_traj, ankle_traj]

# Initial PID Params
k_hip = [1000, 100]
k_knee = [3000, 300]
k_ankle = [1000, 100]
hip_goal = 0
knee_goal = human.joint_limits['knee'][0]
ankle_goal = human.joint_limits['ankle'][1]

pub_goal = rospy.Publisher('goal', Float32MultiArray, queue_size=1)


def init_k_vals():
    """
    set the k values for each joint based on the parameter
    recreates the Controller
    """
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
def loop(tf=5):
    Controller = init_k_vals()     # initialize the controller values

    hip_traj.create_traj(0, hip_goal, 0, 0, tf)
    knee_traj.create_traj(0, knee_goal, 0, 0, tf)
    ankle_traj.create_traj(0, -0.3491, 0, 0, tf)

    start = rospy.get_time()
    t = 0
    msg = Float32MultiArray()

    # run the controller for the given time
    print("Starting loop")
    while abs(t) < tf:
        t = rospy.get_time() - start

        q_goal = np.array([0.0] * num_joints)
        qd_goal = np.array([0.0] * num_joints)
        qdd_goal = np.array([0.0] * num_joints)

        # get traj value
        for i in range(len(trajs)):
            traj_q, traj_qd, traj_qdd = trajs[i].get_traj(t)   # values from traj for specific joint

            # set the traj values to the correct joint state
            q_goal[left_order[leg_segs[i]]] = traj_q
            q_goal[right_order[leg_segs[i]]] = traj_q
            qd_goal[left_order[leg_segs[i]]] = traj_qd
            qd_goal[right_order[leg_segs[i]]] = traj_qd
            qdd_goal[left_order[leg_segs[i]]] = traj_qdd
            qdd_goal[right_order[leg_segs[i]]] = traj_qdd

        # Get current states
        msg.data = q_goal
        pub_goal.publish(msg)

        q = human.q
        qd = human.qd
        aq = qdd_goal + Controller.get_tau(q_goal - q, qd_goal - qd)
        # print('q')
        # print(q)
        # Calc tau from dynamical model
        tau = human.calculate_dynamics(aq)
        # print('tau')
        # print(tau)
        human.update_torque(tau)
        t = rospy.get_time() - start

    print("5 second loop over")


def set_body(h=.2,r=1.3):
    """
    Set body position to standing
    """
    body.set_pos(0, 0, h)
    body.set_rpy(r, 0, 0)


def free_body():
    """
    undo the 'set_body' and remove all efforts
    """
    print("removing position setpoints oh boy")
    body._cmd.enable_position_controller = False
    remove_torques()


def remove_torques():
    for i in range(len(body.get_joint_names())):
        body.set_joint_effort(i,0)


set_body()


