import rospy
import numpy as np
import ambf_msgs.msg as ambf
from Model import Exoskeleton


class AMBF(Exoskeleton.Exoskeleton):

    def __init__(self, name_space, mass, height):
        """

        :param name_space: name space of the the body
        """

        super(AMBF, self).__init__(mass, height)
        rospy.init_node("AMBF_Walker")
        self.time = 0
        self.sub = rospy.Subscriber(name_space + "/body/State", ambf.ObjectState, self.joint_callback)
        self.tau_pub = rospy.Publisher(name_space + "/body/Command", ambf.ObjectCmd, queue_size=1)

    def send_command(self, tau):
        cmd = ambf.ObjectCmd()
        cmd.joint_cmds = tau
        self.tau_pub.publish(cmd)


    def joint_callback(self, msg):
        """
        callback fnc for the joint listener
        :type msg: ambf.ObjectState
        :param msg:
        :return:
        """

        # this checks if we have the joint states enable and if they are
        # not then they are are enables
        if not msg.joint_positions:
            msg = ambf.ObjectCmd()
            msg.publish_joint_names = True
            msg.publish_joint_positions = True
            self.tau_pub.publish(msg)
            return None

        q = np.array([0.0] * 6)
        qd = np.array([0.0] * 6)

        q = msg.joint_positions
        q = np.round(q, 3)

        if self.time == 0:
            self.time = msg.sim_time

        qd = (q - self.q) / (msg.sim_time - self.time + 0.0000001)

        self.update_joints(q, qd)

