import rospy
from Simulation import AMBF
from Utlities import Read_Mocap
from Controller import PD_Controller
import numpy as np
from std_msgs.msg import Float64
import time as clock
from lib.Python import RMP_runner
from Tkinter import *



def calculate_gain(Ku, Tu):
    Td = Tu / 8.0
    Kp = 0.8 * Ku
    Kd = (Ku * Tu) / 10.0
    return Kp, Kd



def go():

    Kp = np.array([0, sliders[0][0].get(),  sliders[1][0].get(),  sliders[2][0].get(), 0, 0, 0])
    Kd = np.array([0, sliders[0][1].get(),  sliders[1][1].get(),  sliders[2][1].get(), 0, 0, 0])

    Controller.Kd = Kd
    Controller.Kp = Kp
    traj_h = Float64()
    traj_k = Float64()
    traj_a = Float64()
    q = sim.q
    qd = sim.qd

    hip, hipd, hipdd = hip_runner.step(1.0)
    knee, kneed, kneedd = knee_runner.step(1.0)
    ankle, ankled, ankledd = ankle_runner.step(1.0)
    q_goal[1] = hip
    qd_goal[1] = hipd
    qdd_goal[1] = hipdd

    q_goal[2] = knee
    qd_goal[2] = kneed
    qdd_goal[2] = kneedd

    q_goal[3] = ankle
    qd_goal[3] = ankled
    qdd_goal[3] = ankledd

    aq = qdd_goal + Controller.calc(q_goal - q, qd_goal - qd)
    tau = sim.calculate_dynamics(q_d, qd_d, aq)

    for i in xrange(0, 6):
        cmd[i] = tau[i + 1]

    sim.send_command(cmd)
    traj_h.data = q_goal[1]
    traj_k.data = q_goal[2]
    traj_a.data = q_goal[3]

    traj_hip.publish(traj_h)
    traj_knee.publish(traj_k)
    traj_ankle.publish(traj_a)

    dt = sim.dt
    root.after(10,go)


sim = AMBF.AMBF("revolute", 52, 1.57)
hip_runner = RMP_runner.RMP_runner("/home/nathaniel/git/AMBF_Walker/config/hip.xml")
knee_runner = RMP_runner.RMP_runner("/home/nathaniel/git/AMBF_Walker/config/knee.xml")
ankle_runner = RMP_runner.RMP_runner("/home/nathaniel/git/AMBF_Walker/config/ankle.xml")


traj_hip = rospy.Publisher("traj_hip", Float64, queue_size=1)
traj_knee = rospy.Publisher("traj_knee", Float64, queue_size=1)
traj_ankle = rospy.Publisher("traj_ankle", Float64, queue_size=1)

q_d = np.asarray([0.0] * 7)
qd_d = np.asarray([0.0] * 7)
qdd_d = np.asarray([0.0] * 7)
q_goal = np.asarray([0.0] * 7)
qd_goal = np.asarray([0.0] * 7)
qdd_goal = np.asarray([0.0] * 7)
cmd = np.asarray([0.0] * 6)
dt = 0

root = Tk()

Kp_hip, Kd_hip = calculate_gain(150.0, 0.6)
Kp_knee, Kd_knee = calculate_gain(75.0, 1.05)
Kp_ankle, Kd_ankle = calculate_gain(260.0, 0.55)
gains = ( [ 437, 11 ],[440, 12],[Kp_ankle, Kd_ankle])

Kp = np.array([0, 437, 440, Kp_ankle, Kp_hip, Kp_knee, Kp_ankle])
Kd = np.array([0, 11, 12, Kd_ankle, Kd_hip, Kd_knee, Kd_ankle])
Controller = PD_Controller.PDController(Kp, Kd)
sliders = []
for i in xrange(0,3):
    joint_gains = gains[i]
    P = Scale(root, from_=0, to=2500,length=1200, resolution=1, orient=HORIZONTAL)
    D = Scale(root, from_=0, to=200,length=1200, resolution=0.1, orient=HORIZONTAL)
    P.set(joint_gains[0])
    D.set(joint_gains[1])
    P.pack()
    D.pack()
    sliders.append((P,D))


if __name__ == "__main__":
    root.after(1, go)
    root.mainloop()









