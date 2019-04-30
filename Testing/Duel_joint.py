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

    sim = AMBF.AMBF("revolute", 52, 1.57)
    # plot = Plotter.Plotter(sim)
    pub = rospy.Publisher("traj",Float64, queue_size=1)
    cmd = np.asarray([0.0] * 6)
    q_d = np.asarray([0.0] * 7)
    qd_d = np.asarray([0.0] * 7)
    qdd_d = np.asarray([0.0] * 7)
    Ku = 50.0
    Tu = 1.60
    Td = Tu/8.0
    Kp = 0.8*Ku
    Kd = (Ku*Tu)/10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller1 = PD_Controller.PDController(kp, kd)
    Ku = 50.0
    Tu = 1.60
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller2 = PD_Controller.PDController(kp, kd)

    start = 0
    end = -0.3
    total_time = 3.0
    coef1 = get_coef(start, end, total_time)

    start = 0
    end = 0.3
    total_time = 3.0
    coef2 = get_coef(start, end, total_time)
    time = 0

    while time <= total_time:

        dt = sim.dt
        q = sim.q
        qd = sim.qd

        q1_goal = (coef1[0] + coef1[1] * time + coef1[2] * time ** 2 + coef1[3] * time ** 3)[0]
        qd1_goal = (coef1[1] + 2 * coef1[2] * time + 3 * coef1[3] * time ** 2)[0]
        qdd1_goal = (2 * coef1[2] + 6 * coef1[3] * time)[0]
        qdd1 = controller1.calc(q1_goal - q[1], qd1_goal - qd[1])

        q2_goal = (coef2[0] + coef2[1] * time + coef2[2] * time ** 2 + coef2[3] * time ** 3)[0]
        qd2_goal = (coef2[1] + 2 * coef2[2] * time + 3 * coef2[3] * time ** 2)[0]
        qdd2_goal = (2 * coef2[2] + 6 * coef2[3] * time)[0]
        qdd2 = controller1.calc(q2_goal - q[2], qd2_goal - qd[2])

        traj = Float64()
        traj.data = q1_goal

        q_d[1] = q1_goal
        qd_d[1] = qd1_goal
        qdd_d[1] = qdd1[0]

        tau = sim.calculate_dynamics(q_d, qd_d, qdd_d)
        cmd = tau
        sim.send_command(cmd)
        time += dt
        #plot.update()
        pub.publish(traj)
        clock.sleep(dt)

    while 1:
        sim.send_command(cmd)



