import numpy as np
import rospy
from Simulation import AMBF
from Controller import PD_Controller
import numpy as np
from Utlities import Plotter
import time as clock
from std_msgs.msg import Float64


def get_coef(start, end, dt):
    b = np.array([[start], [0], [end], [0]])

    A = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [1, dt, dt ** 2, dt ** 3],
                  [0, 1, 2 * dt, 3 * dt ** 2]])
    return np.matmul(np.linalg.pinv(A), b)


if __name__ == "__main__":

    total_time = 3.0
    sim = AMBF.AMBF("revolute", 52, 1.57)
    # plot = Plotter.Plotter(sim)
    traj_hip = rospy.Publisher("traj_hip", Float64, queue_size=1)
    traj_knee = rospy.Publisher("traj_knee", Float64, queue_size=1)
    traj_ankle = rospy.Publisher("traj_ankle", Float64, queue_size=1)
    err = rospy.Publisher("error", Float64, queue_size=1)
    cmd = np.asarray([0.0] * 6)

    q_d = np.asarray([0.0] * 6)
    qd_d = np.asarray([0.0] * 6)
    qdd_d = np.asarray([0.0] * 6)

    Ku = 180.0
    Tu = 0.3
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller_hip = PD_Controller.PDController(kp, kd)
    start = 0
    end = -0.3
    coef_hip = get_coef(start, end, total_time)

    Ku = 280.0
    Tu = 0.3
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller_knee = PD_Controller.PDController(kp, kd)
    start = 0
    end = 1.0
    coef_knee = get_coef(start, end, total_time)

    Ku = 249.0
    Tu = 0.475
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller_ankle = PD_Controller.PDController(kp, kd)
    start = 0.873
    end = 0.0
    coef_ankle = get_coef(start, end, total_time)

    time = 0
    while time <= total_time:
        side = 0
        traj_h = Float64()
        traj_k = Float64()
        traj_a = Float64()
        dt = sim.dt
        q = sim.q
        qd = sim.qd

        q_goal = (coef_hip[0] + coef_hip[1] * time + coef_hip[2] * time ** 2 + coef_hip[3] * time ** 3)[0]
        qd_goal = (coef_hip[1] + 2 * coef_hip[2] * time + 3 * coef_hip[3] * time ** 2)[0]
        qdd_goal = (2 * coef_hip[2] + 6 * coef_hip[3] * time)[0]
        qdd_hip = qdd_goal + controller_hip.calc(q_goal - q[0+side], qd_goal - qd[0+side])
        q_d[0+side] = q_goal
        qd_d[0+side] = qd_goal
        qdd_d[0+side] = qdd_hip[0]
        traj_h.data = q_goal

        q_goal = (coef_knee[0] + coef_knee[1] * time + coef_knee[2] * time ** 2 + coef_knee[3] * time ** 3)[0]
        qd_goal = (coef_knee[1] + 2 * coef_knee[2] * time + 3 * coef_knee[3] * time ** 2)[0]
        qdd_goal = (2 * coef_knee[2] + 6 * coef_knee[3] * time)[0]
        qdd_knee = qdd_goal + controller_knee.calc(q_goal - q[1+side], qd_goal - qd[1+side])
        q_d[1+side] = q_goal
        qd_d[1+side] = qd_goal
        qdd_d[1+side] = qdd_knee[0]
        traj_k.data = q_goal

        q_goal = (coef_ankle[0] + coef_ankle[1] * time + coef_ankle[2] * time ** 2 + coef_ankle[3] * time ** 3)[0]
        qd_goal = (coef_ankle[1] + 2 * coef_ankle[2] * time + 3 * coef_ankle[3] * time ** 2)[0]
        qdd_goal = (2 * coef_ankle[2] + 6 * coef_ankle[3] * time)[0]
        qdd_ankle = qdd_goal + controller_ankle.calc(q_goal - q[2+side], qd_goal - qd[2+side])
        q_d[2+side] = q_goal
        qd_d[2+side] = qd_goal
        qdd_d[2+side] = qdd_ankle[0]
        traj_a.data = q_goal
        tau = sim.calculate_dynamics(q_d, qd_d, qdd_d)
        cmd[0+side] = tau[0+side]
        cmd[1+side] = tau[1]
        cmd[2+side] = tau[2]
        sim.send_command(cmd)
        time += dt

        traj_hip.publish(traj_h)
        traj_knee.publish(traj_k)
        traj_ankle.publish(traj_a)

        clock.sleep(dt)

    while 1:
        sim.send_command(cmd)
        traj_hip.publish(traj_h)
        traj_knee.publish(traj_k)
        traj_ankle.publish(traj_a)

    #
    #
