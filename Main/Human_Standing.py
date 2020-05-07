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
orders = [human.ambf_order_crutch_left, human.ambf_order_crutch_right]

children = body.get_children_names()
print(children)

# Targets joint values
num_joints = len(children)

# controller inits
Controller = PDController(0, 0)
trajs = [[TrajectoryGen(), TrajectoryGen(), TrajectoryGen()],
         [TrajectoryGen(), TrajectoryGen(), TrajectoryGen()]
]

# Initial PID Params
Ks = [
    [100, 10],
    [100, 10],
    [80, 5],
]
goals = [
    [-1.1],
    [1.9],
    [-0.76]
]

pub_goal = rospy.Publisher('goal', Float32MultiArray, queue_size=1)


def init_k_vals():
    """
    set the k values for each joint based on the parameter
    recreates the Controller
    """
    K_mat = np.zeros((2,num_joints,num_joints))
    for i in range(2):
        for j in range(len(orders)):
            for k in range(len(leg_segs)):
                idx = orders[j][leg_segs[k]]
                K_mat[i,idx,idx] = Ks[k][i]

    Controller = PDController(K_mat[0,...], K_mat[1,...])
    return Controller


# Loop to stay standing
def loop(tf=5):
    Controller = init_k_vals()     # initialize the controller values

    for j in range(len(trajs[0])):
        trajs[0][j].create_traj(0, goals[j][0], 0, 0, tf)

    start = rospy.get_time()
    t = 0

    rate = rospy.Rate(1000)  # 1000hz

    # run the controller for the given time
    print("Starting loop")
    while abs(t) < tf:
        t = control_loop(start, Controller)
        rate.sleep()
    print("5 second loop over")


def control_loop(start, Controller):
    t = rospy.get_time() - start

    goals = np.zeros((3,num_joints))
    msg = Float32MultiArray()

    # get traj value
    for j in range(len(trajs[0])):
        traj_data = trajs[0][j].get_traj(t)  # values from traj for specific joint

        # set the traj values to the correct joint state
        goals[0,orders[0][leg_segs[j]]] = traj_data[0]
        goals[1,orders[0][leg_segs[j]]] = traj_data[1]
        goals[2,orders[0][leg_segs[j]]] = traj_data[2]
        goals[0,orders[1][leg_segs[j]]] = [-0.05,0,-0.1][j]
        goals[1,orders[1][leg_segs[j]]] = 0
        goals[2,orders[1][leg_segs[j]]] = 0

    # publish
    msg.data = goals[0,:]
    pub_goal.publish(msg)

    # Get current states
    q = human.q
    qd = human.qd
    aq = goals[2,:] + Controller.get_tau(goals[0,:] - q, goals[1,:] - qd)

    # Calc tau from dynamical model
    tau = human.calculate_dynamics(aq)
    human.update_torque(tau)
    t = rospy.get_time() - start
    return t


def set_body(h=.2, r=math.pi/2+0.15):
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

    for j in range(len(trajs[0])):
        trajs[0][j].create_traj(0, 0, 0, 0, tf)

    # run the controller for the given time
    print("Starting loop")
    rate = rospy.Rate(1000)  # 1000hz
    while abs(t) < tf:
        if h >= 0:
            h = .2 - dh*t
            set_body(h)
        elif h <= 0 and h is not -1:
            print('At ground, free body')
            #free_body()
            h = -1

        t = control_loop(start, Controller)
        rate.sleep()
    print("loop done")
    #remove_torques()

    for j in range(len(trajs[0])):
        trajs[0][i].create_traj(0, goals[j][0], 0, 0, tf)

    while abs(t) < tf:
        t = control_loop(start, Controller)
        rate.sleep()

set_body()
