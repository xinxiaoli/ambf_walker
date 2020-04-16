import sys
from ambf_client import Client
from Controller.PDController import PDController
import numpy as np
import time
from Model.Human import Human
import ambf_msgs.msg as ambf


# Create Client and connect
_client = Client()
_client.connect()
print(_client.get_obj_names())

# body_handle = _client.get_obj_handle('body')
# h = Human(_client, 50, 1.5)
q_goal = [0.0] * 7
qd_goal = [0.0] * 7
qdd_goal = [0.0] * 7
cmd = np.asarray([0.0] * 6)

gains = ( [ 510,36.8],[565,42.89],[354,45.2],[ 510,36.8],[565,42.89],[354,45.2])

Kp = np.array([0,gains[0][0],gains[1][0],gains[2][0],gains[3][0],gains[4][0],gains[5][0] ])
Kd = np.array([0,gains[0][1],gains[1][1],gains[2][1],gains[3][1],gains[4][1],gains[5][1] ])

Controller = PDController(Kp, Kd)


def go(count):

    q = h.q
    qd = h.qd

    for i in range(6):
        q_goal[i] = 0
        qd_goal[i] = 0
        qdd_goal[i] = 0

    aq = qdd_goal + Controller.calc(q_goal - q, qd_goal - qd)
    tau = h.calculate_dynamics(aq)

    for i in range(6):
        cmd[i] = tau[i + 1]

    return cmd











