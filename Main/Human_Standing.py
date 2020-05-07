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
import math

# Create Client and connect
_client = Client()
_client.connect()
rospy.sleep(1)

human = Human(_client, 50.0, 1.5)
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
k_hip = [100, 10]
k_knee = [100, 10]
k_ankle = [80, 5]
hip_goal = -1.1
knee_goal = 1.9
ankle_goal = -0.32

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
    ankle_traj.create_traj(0, ankle_goal, 0, 0, tf)

    start = rospy.get_time()
    t = 0

    # run the controller for the given time
    print("Starting loop")
    while abs(t) < tf:
        t = control_loop(start, Controller)
    print("5 second loop over")


def control_loop(start, Controller):
    t = rospy.get_time() - start

    q_goal = np.array([0.0] * num_joints)
    qd_goal = np.array([0.0] * num_joints)
    qdd_goal = np.array([0.0] * num_joints)
    msg = Float32MultiArray()

    # get traj value
    for i in range(len(trajs)):
        traj_q, traj_qd, traj_qdd = trajs[i].get_traj(t)  # values from traj for specific joint

        # set the traj values to the correct joint state
        q_goal[left_order[leg_segs[i]]] = traj_q
        q_goal[right_order[leg_segs[i]]] = [-0.05,0,-0.1][i]
        qd_goal[left_order[leg_segs[i]]] = traj_qd
        qd_goal[right_order[leg_segs[i]]] = 0
        qdd_goal[left_order[leg_segs[i]]] = traj_qdd
        qdd_goal[right_order[leg_segs[i]]] = 0

    # publish
    msg.data = q_goal
    pub_goal.publish(msg)

    # Get current states
    q = human.q
    qd = human.qd
    aq = qdd_goal + Controller.get_tau(q_goal - q, qd_goal - qd)

    # Calc tau from dynamical model
    tau = human.calculate_dynamics(aq)
    human.update_torque(tau)
    t = rospy.get_time() - start
    return t


def set_body(h=.2, r=math.pi/2):
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


def slowly_lower(tf=10):
    """
    start body above ground then slowly lower until feet are touching (0,0,0)
    release position controller and continue control loop
    """
    h = .2
    set_body(h)
    start = rospy.get_time()
    t = 0
    dh = h/5

    Controller = init_k_vals()  # initialize the controller values

    hip_traj.create_traj(0, hip_goal, 0, 0, tf)
    knee_traj.create_traj(0, knee_goal, 0, 0, tf)
    ankle_traj.create_traj(0, ankle_goal, 0, 0, tf)

    # run the controller for the given time
    print("Starting loop")
    while abs(t) < tf:
        if h >= 0:
            h = .2 - dh*t
            set_body(h)
        elif h <= 0 and h is not -1:
            print('At ground, free body')
            free_body()
            h = -1

        t = control_loop(start, Controller)
    print("loop done")
    remove_torques()


set_body()
