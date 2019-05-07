import rospy
from Simulation import AMBF
from Utlities import Read_Mocap
from Controller import PD_Controller
import numpy as np
from std_msgs.msg import Float64
import time as clock


def calculate_gain(Ku, Tu):
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    return Kp, Kd


if __name__ == "__main__":

    sim = AMBF.AMBF("revolute", 52, 1.57)
    file_path = "/home/nathaniel/git/AMBF_Walker/config/joint_data.csv"
    Kp_hip, Kd_hip = calculate_gain(180.0, 0.3)
    Kp_knee, Kd_knee = calculate_gain(1000.0, .0)
    Kp_ankle, Kd_ankle = calculate_gain(249.0, 0.475)
    Kp = np.array([0,Kp_hip, Kp_knee, Kp_ankle, Kp_hip, Kp_knee, Kp_ankle])
    Kd = np.array([0,Kd_hip, Kp_knee, Kd_ankle, Kd_hip, Kd_knee, Kd_ankle])
    Controller = PD_Controller.PDController(Kp, Kd)
    Traj = Read_Mocap.Read_Mocap(file_path)

    traj_hip = rospy.Publisher("traj_hip", Float64, queue_size=1)
    traj_knee = rospy.Publisher("traj_knee", Float64, queue_size=1)
    traj_ankle = rospy.Publisher("traj_ankle", Float64, queue_size=1)

    q_d = np.asarray([0.0] * 7)
    qd_d = np.asarray([0.0] * 7)
    qdd_d = np.asarray([0.0] * 7)
    cmd = np.asarray([0.0] * 6)
    dt = 0
    from Tkinter import *



    mainloop()
    while 1:
        traj_h = Float64()
        traj_k = Float64()
        traj_a = Float64()
        q = sim.q
        qd = sim.qd

        q_goal, qd_goal = Traj.get_next_point(dt)

        qdd_d = Controller.calc(q_goal - q, qd_goal - qd)
        tau = sim.calculate_dynamics(q_d, qd_d, qdd_d)

        for i in xrange(0,6):
            cmd[i] = tau[i+1]

        sim.send_command(cmd)
        traj_h.data = q_goal[1]
        traj_k.data = q_goal[2]
        traj_a.data = q_goal[3]

        traj_hip.publish(traj_h)
        traj_knee.publish(traj_k)
        traj_ankle.publish(traj_a)

        dt = sim.dt
        clock.sleep(dt)








