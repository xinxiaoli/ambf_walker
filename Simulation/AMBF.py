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

        joint_names = msg.joint_names
        joint_angles = msg.joint_positions

        q_raw = {}
        q_mean = {}
        qd_mean = {}

        if self.time == 0:
            self.time = msg.sim_time

        for name, angle in zip(joint_names, joint_angles):
            q_raw[name] = round(angle,2)

        qd_raw = self.calculate_qd(q_raw, msg.sim_time)

        for name in joint_names:
            q_mean[name] = q_raw[name]
            qd_mean[name] = qd_raw[name]

        self.q = q_raw
        self.qd = qd_raw
        
    def calculate_qd(self, q, current_time):
        """
        Calculate the joint velocity
        :type current_time: float
        :param current_time: time
        :type q: dict
        """
        qd = {}

        # check is the dict is empty
        if not self._q:
            self._q = q

        for key, angle in q.iteritems():
            qd[key] = (angle - self._q[key] ) / (current_time - self.time + 0.0000001)

        return qd


