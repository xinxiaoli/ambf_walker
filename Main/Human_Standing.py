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

# Each row is for each joint
# Kp constants are first, Kv constants are second
Ks = [
    [110, 11],
    [120, 12],
    [80, 8],
]

# The first group is for the left leg, the second is for the right leg
# Each row of the block is for each joint of the leg
# Each element in that row is then a list of the setpoints
goals_walking = [
    [
        [-0.07, -1.12,  -0.9],
        [    0,   1.9,  0.15],
        [ -0.1, -0.78,   0.8]
    ],
    [
        [-0.07, -0.12,  0.22],
        [    0,     0,     0],
        [ -0.1,  -0.1,  -0.2]
    ]
]

# Standing
goals_straight = np.zeros((3,3,2))

goals_bent = [
    [
        [0, 0],
        [.5, .5],
        [0,0]
    ],
    [
        [0,0],
        [1,1],
        [0,0]
    ]
]

pub_goal = rospy.Publisher('goal', Float32MultiArray, queue_size=1)


def init_k_vals():
    """
    set the k values for each joint based on the parameter
    recreates the Controller
    """
    K_mat = np.zeros((2,num_joints,num_joints))
    # For both Kp and Kv
    for i in range(2):
        for side in range(len(orders)):
            for joint in range(len(leg_segs)):
                # Which cell in the matrix is this one
                idx = orders[side][leg_segs[joint]]
                # Create the diagonal matrix as we go
                K_mat[i,idx,idx] = Ks[joint][i]

    Controller = PDController(K_mat[0,...], K_mat[1,...])
    return Controller


# Loop to stay standing
def loop(tf=5, goals=goals_bent):
    trajs_loop = [[TrajectoryGen(), TrajectoryGen(), TrajectoryGen()],
             [TrajectoryGen(), TrajectoryGen(), TrajectoryGen()]
             ]

    Controller = init_k_vals()     # initialize the controller values
    print("Starting loop")
    for waypoint in range(1,len(goals[0][0])):
        for side in range(len(orders)):
            for joint in range(len(trajs_loop[0])):
                trajs_loop[side][joint].create_traj(goals[side][joint][waypoint-1], goals[side][joint][waypoint], 0, 0, tf)

        start = rospy.get_time()
        t = 0

        rate = rospy.Rate(1000)  # 1000hz

        # run the controller for the given time
        print("going to waypoint {0}".format((waypoint-1)))

        while abs(t) < tf:
            t = control_loop(start, Controller, trajs_loop)
            rate.sleep()
        print("{0} second loop over".format(tf))
    print("all waypoints reached")


def control_loop(start, Controller, trajectory):
    t = rospy.get_time() - start

    goal_qs = np.zeros((3,num_joints))
    msg = Float32MultiArray()

    # get traj value
    for side in range(len(orders)):
        for joint in range(len(trajectory[0])):
            traj_data = trajectory[side][joint].get_traj(t)  # values from traj for specific joint
            idx = orders[side][leg_segs[joint]]
            # Iterate through the q,qd,qdd for each trajectory
            for k in range(len(traj_data)):
                goal_qs[k,idx] = traj_data[k]

    # publish
    msg.data = goal_qs[0,:]
    pub_goal.publish(msg)

    # Get current states
    q = human.q
    qd = human.qd
    aq = goal_qs[2,:] + Controller.get_tau(goal_qs[0,:] - q, goal_qs[1,:] - qd)

    # Calc tau from dynamical model
    tau = human.calculate_dynamics(aq)
    human.update_torque(tau)
    t = rospy.get_time() - start
    return t


def set_body(remove_t = False, h=.2, r=math.pi/2+0.08):
    """
    Set body position to standing
    """
    body.set_pos(0, 0, h)
    body.set_rpy(r, 0, 0)

    if remove_t:
        remove_torques()


def free_body():
    """
    undo the 'set_body' and remove all efforts
    """
    print("removing position setpoints oh boy")
    body._cmd.enable_position_controller = False
    #remove_torques()


def remove_torques():
    for i in range(len(body.get_joint_names())):
        body.set_joint_effort(i,0)


def slowly_lower(tf=10):
    """
    start body above ground then slowly lower until feet are touching (0,0,0)
    release position controller and continue control loop
    """
    h = .2
    set_body(h=h)
    start = rospy.get_time()
    t = 0
    dh = h/5

    Controller = init_k_vals()  # initialize the controller values

    trajs = [[TrajectoryGen(), TrajectoryGen(), TrajectoryGen()],
             [TrajectoryGen(), TrajectoryGen(), TrajectoryGen()]
             ]

    for side in range(len(orders)):
        for joint in range(len(trajs[0])):
            trajs[side][joint].create_traj(0, 0, 0, 0, tf)

    # run the controller for the given time
    print("Starting loop")
    rate = rospy.Rate(1000)  # 1000hz
    while abs(t) < 6:
        if h >= 0:
            h = .2 - dh*t
            set_body(h=h)
        elif h <= 0 and h is not -1:
            print('At ground, free body')
            h = -1
            free_body()

        t = control_loop(start, Controller, trajs)
        rate.sleep()
    print("loop done")

    loop(tf)

set_body()
