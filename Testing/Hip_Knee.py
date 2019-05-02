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
  
    total_time = 3.0
    sim = AMBF.AMBF("revolute", 52, 1.57)
    # plot = Plotter.Plotter(sim)
    traj_hip = rospy.Publisher("traj_hip",Float64, queue_size=1)
    traj_knee = rospy.Publisher("traj_knee", Float64, queue_size=1)
    err = rospy.Publisher("error", Float64, queue_size=1)
    cmd = np.asarray([0.0] * 6)

    q_d = np.asarray([0.0] * 6)
    qd_d = np.asarray([0.0] * 6)
    qdd_d = np.asarray([0.0] * 6)
  
    Ku = 65.0
    Tu = 1.0
    Td = Tu/8.0
    Kp = Ku
    Kd = (Ku*Tu)/10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller_hip = PD_Controller.PDController(kp, kd)
    start = 0
    end = -0.3
    coef_hip = get_coef(start, end, total_time)
    

    Ku = 0.285
    Tu = 2.25
    Td = Tu / 8.0
    Kp = Ku
    Kd = (Ku * Tu) / 10.0
    kp = np.array([Kp])
    kd = np.array([Kd])
    controller_knee = PD_Controller.PDController(kp, kd)  
    start = 0
    end = -0.20
    coef_knee = get_coef(start, end, total_time)
    
    time = 0
    while time <= total_time:

        traj_h = Float64()
        traj_k = Float64()
        dt = sim.dt
        q = sim.q
        qd = sim.qd

        q_goal = (coef_hip[0] + coef_hip[1] * time + coef_hip[2] * time ** 2 + coef_hip[3] * time ** 3)[0]
        qd_goal = (coef_hip[1] + 2 * coef_hip[2] * time + 3 * coef_hip[3] * time ** 2)[0]
        qdd_goal = (2 * coef_hip[2] + 6 * coef_hip[3] * time)[0]
        qdd_hip = qdd_goal + controller_hip.calc(q_goal - q[0], qd_goal - qd[0])
        q_d[0] = q_goal
        qd_d[0] = qd_goal
        qdd_d[0] = qdd_hip[0]
        traj_h.data = q_goal

        q_goal = (coef_knee[0] + coef_knee[1] * time + coef_knee[2] * time ** 2 + coef_knee[3] * time ** 3)[0]
        qd_goal = (coef_knee[1] + 2 * coef_knee[2] * time + 3 * coef_knee[3] * time ** 2)[0]
        qdd_goal = (2 * coef_knee[2] + 6 * coef_knee[3] * time)[0]
        qdd_knee = qdd_goal + controller_knee.calc(q_goal - q[1], qd_goal - qd[1])
        q_d[1] = q_goal
        qd_d[1] = qd_goal
        qdd_d[1] = qdd_knee[0]
        traj_k.data = q_goal

        tau = sim.calculate_dynamics(q_d, qd_d, qdd_d)

        cmd[0] = tau[0]
        cmd[1] = -tau[1]
        sim.send_command(cmd)
        time += dt
        #plot.update()

        traj_hip.publish(traj_h)
        traj_knee.publish(traj_k)

        clock.sleep(dt)

    while 1:
        sim.send_command(cmd)
        traj_hip.publish(traj_h)
        traj_knee.publish(traj_k)



