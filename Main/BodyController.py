import rospy
import ambf_msgs.msg as ambf
import math
import numpy as np
from geometry_msgs.msg import Pose, Wrench
from Controller import PD_Controller

class PlaneController(object):

    def __init__(self):

        rospy.init_node("plane_controller")
        rospy.Subscriber("joint_cmd", ambf.ObjectCmd, self.controller_cb)
        rospy.Subscriber("revolute/body/State", ambf.ObjectState, self.state_callback)
        self.pub = rospy.Publisher("revolute/body/Command", ambf.ObjectCmd, queue_size=1)
        self.position = np.asarray([0, 0, 0])
        self.velocity = np.asarray([0, 0, 0])
        self.prevous_position = np.asarray([0, 0, 0])
        self.have_state = False
        self.prevous_time = 0
        kp = np.array([500, 500, 0])
        kd = np.array([0.25, 0.25, 0])
        self.controller = PD_Controller.PDController(kp, kd)
        self.time = 0


    def controller_cb(self, msg):
        """

        :type msg: ambf.ObjectCmd
        """

        pos_ref = np.asarray([0, 0, 0])
        vel_ref = np.asarray([0, 0, 0])
        e = pos_ref - self.position
        ed = vel_ref - self.velocity
        force = self.controller.calc(e,ed)
        cmd = msg
        cmd.wrench.force.x = force[0]
        cmd.wrench.force.y = force[1]
        cmd.wrench.force.z = 0#force[2]
        print cmd
        self.pub.publish(cmd)

    def state_callback(self, msg):

        self.position = np.asarray([msg.pose.position.x,msg.pose.position.y, msg.pose.position.z] )

        if self.time == 0:
            self.time = msg.wall_time
        dt = (msg.wall_time - self.time) + 0.0000000001
        self.time = msg.wall_time

        self.velocity = (self.position - self.prevous_position) / dt
        self.velocity = np.clip(self.velocity, -2.0, 2.0)
        self.prevous_position = self.position


if __name__ == "__main__":
    PlaneController()
    rospy.spin()
