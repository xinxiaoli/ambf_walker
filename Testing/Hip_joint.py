import numpy as np
import rospy
from Simulation import AMBF
from Controller import PD_Controller
import numpy as np
from Utlities import Plotter
import time as clock
from std_msgs.msg import Float64

def get_coef( start, end, dt):

    b = np.array([[start], [0], [end], [0]])

    A = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [1, dt, dt ** 2, dt ** 3],
                  [0, 1, 2 * dt, 3 * dt ** 2]])
    return np.matmul(np.linalg.pinv(A), b)


if __name__ == "__main__":
    joint = 0
    sim = AMBF.AMBF("revolute", 52, 1.57)
    plot = Plotter.Plotter(sim)
    pub = rospy.Publisher("traj_hip",Float64, queue_size=1)
    err = rospy.Publisher("error", Float64, queue_size=1)
    cmd = np.asarray([0.0] * 6)

    q_d = np.asarray([0.0] * 6)
    qd_d = np.asarray([0.0] * 6)
    qdd_d = np.asarray([0.0] * 6)
    Ku = 180.0
    Tu = 0.3
    Td = Tu/8.0
    Kp = 0.8*Ku
    Kd = (Ku*Tu)/10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller = PD_Controller.PDController(kp, kd)

    start = 0
    total_time = 3.0
    end = -0.3
    coef = get_coef(start, end, total_time)
    time = 0

    while 1:
        joint = 0
        dt = sim.dt
        q = sim.q
        qd = sim.qd

        q_goal = (coef[0] + coef[1] * time + coef[2] * time ** 2 + coef[3] * time ** 3)[0]
        qd_goal = (coef[1] + 2 * coef[2] * time + 3 * coef[3] * time ** 2)[0]
        qdd_goal = (2 * coef[2] + 6 * coef[3] * time)[0]
        qdd = qdd_goal + controller.calc(q_goal - q[joint], qd_goal - qd[joint])

        q_goal = (coef[0] + coef[1] * time + coef[2] * time ** 2 + coef[3] * time ** 3)[0]
        qd_goal = (coef[1] + 2 * coef[2] * time + 3 * coef[3] * time ** 2)[0]
        qdd_goal = (2 * coef[2] + 6 * coef[3] * time)[0]
        qdd = qdd_goal + controller.calc(q_goal - q[3], qd_goal - qd[3])



        q_d[joint] = q_goal
        qd_d[joint] = qd_goal
        qdd_d[joint] = qdd[0]
        tau = sim.calculate_dynamics(q_d, qd_d, qdd_d)
        print tau
        cmd[0] = tau[joint]
        sim.send_command(cmd)
        time += dt
        plot.update()
        #pub.publish(traj)
        clock.sleep(dt)

    # while 1:
    #     sim.send_command(cmd)



