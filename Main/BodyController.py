import rospy
import ambf_msgs.msg as ambf
import math
import numpy as np
from geometry_msgs.msg import Pose, Wrench
from Controller import PD_Controller

class PlaneController(object):

    def __init__(self):

        rospy.init_node("plane_controller")
        rospy.Subscriber("plane_callback", ambf.ObjectCmd, self.position_cb)
        rospy.Subscriber("revolute/body/State", ambf.ObjectState, self.state_callback)
        self.pub = rospy.Publisher("revolute/body/Command", ambf.ObjectCmd, queue_size=1)
        self.position = np.asarray([0, 0, 0])
        self.velocity = np.asarray([0, 0, 0])
        self.prevous_position = np.asarray([0, 0, 0])
        self.have_state = False
        self.prevous_time = 0
        kp = np.array()
        self.controller = PD_Controller.PDController()
        rospy.spin()



    def position_cb(self, msg):

        if not self.have_state: return
        pos_ref = np.asarray([0,0,0])
        vel_ref = np.asarray([0,0,0])

        x = self.position[0]
        y = self.position[1]
        z = self.position[2]

        xd = self.velocity[0]
        yd = self.velocity[1]
        zd = self.velocity[2]









    def state_callback(self, msg):
        have_state = True

        self.position = np.asarray([msg.position.x,msg.position.y, msg.position.z] )

        dt = msg.wall_time - self.time
        self.time = msg.wall_time

        qd = (self.position - self.prevous_position) / dt
        self.velocity = np.clip(qd, -2.0, 2.0)
        self.prevous_position = self.position


if __name__ == "__main__":
    rospy.init_node("plane_controller")
    rospy.Subscriber("plane_callback", ambf.ObjectCmd, position_cb)
    rospy.Subscriber("revolute/body/State", ambf.ObjectState, state_callback)
    pub = rospy.Publisher("revolute/body/Command", ambf.ObjectCmd, queue_size=1)
    rospy.spin()
