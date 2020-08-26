#!/usr/bin/env python3

import rospy
from ambf_msgs.msg import RigidBodyState


class Grabber:

    def __init__(self):
        self.FrontSensorLeftThigh = None
        self.FrontSensorLeftShank = None
        self.FrontSensorRightThigh = None
        self.FrontSensorRightShank = None
        self.BackSensorLeftThigh = None
        self.BackSensorLeftShank = None
        self.BackSensorRightThigh = None
        self.BackSensorRightShank = None
        self.SensorLeftFoot1 = None
        self.SensorLeftFoot2 = None
        self.SensorLeftFoot3 = None
        self.SensorRightFoot1 = None
        self.SensorRightFoot2 = None
        self.SensorRightFoot3 = None

    def callback_leg(data):

        print('FrontSensorLeftThigh')
        print(data.wrench)

    def callback_foot(data):

        print('FrontSensorLeftShank')
        print(data.wrench)


if __name__ == '__main__':
    try:
        rospy.init_node('wrench_subscriber', anonymous=True)

        grabber = Grabber()

        rospy.Subscriber("/ambf/env/FrontSensorLeftThigh/State", RigidBodyState, grabber.callback_leg)
        rospy.Subscriber("/ambf/env/BackSensorLeftThigh/State", RigidBodyState, grabber.callback_leg)

        rospy.Subscriber("/ambf/env/FrontSensorLeftShank/State", RigidBodyState, grabber.callback_leg)
        rospy.Subscriber("/ambf/env/BackSensorLeftShank/State", RigidBodyState, grabber.callback_leg)

        rospy.Subscriber("/ambf/env/FrontSensorRightThigh/State", RigidBodyState, grabber.callback_leg)
        rospy.Subscriber("/ambf/env/BackSensorRightThigh/State", RigidBodyState, grabber.callback_leg)

        rospy.Subscriber("/ambf/env/FrontSensorRightShank/State", RigidBodyState, grabber.callback_leg)
        rospy.Subscriber("/ambf/env/BackSensorRightShank/State", RigidBodyState, grabber.callback_leg)
       
        rospy.Subscriber("/ambf/env/SensorLeftFoot1/State", RigidBodyState, grabber.callback_foot)
        rospy.Subscriber("/ambf/env/SensorLeftFoot2/State", RigidBodyState, grabber.callback_foot)
        rospy.Subscriber("/ambf/env/SensorLeftFoot3/State", RigidBodyState, grabber.callback_foot)
        
        rospy.Subscriber("/ambf/env/SensorRightFoot1/State", RigidBodyState, grabber.callback_foot)
        rospy.Subscriber("/ambf/env/SensorRightFoot2/State", RigidBodyState, grabber.callback_foot)
        rospy.Subscriber("/ambf/env/SensorRightFoot3/State", RigidBodyState, grabber.callback_foot)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
