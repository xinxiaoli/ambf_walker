import rospy
import numpy as np
import ambf_msgs.msg as ambf


class Walker(object):

    def __init__(self, name_space):
        """

        :param name_space: name space of the the body
        """

        rospy.init_node("AMBF_Walker")
        self.time = 0
        self.sub = rospy.Subscriber(name_space + "/body/State", ambf.ObjectState, self.joint_callback)
        self.tau_pub = rospy.Publisher(name_space + "/body/Command", ambf.ObjectCmd, queue_size=1)


    def joint_callback(self, msg):
        """
        callback fnc for the joint listener
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

        if self.time == 0:
            self.time = msg.sim_time

        q = np.array([0.0] * 7)
        qd = np.array([0.0] * 7)

        q[1:] = msg.joint_positions
        q = np.round(q, 2)

        # Correct the joint index location
        q[1], q[3] = q[3], q[1]

        qd = (q - self.q) / (msg.sim_time - self.time + 0.0000001)
        # qd = np.clip(qd, -1, 1)
        for index, (q_, qd_) in enumerate(zip(q, qd)):
            if index == 3 or index == 2 or index == 1:
                self.q[index] = self.q_mean_filter[index].update(q_)
                self.qd[index] = self.qd_mean_filter[index].update(qd_)


