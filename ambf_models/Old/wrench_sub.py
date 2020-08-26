#!/usr/bin/env python3

import rospy
import message_filters
from ambf_msgs.msg import RigidBodyState


def callback_leg(front, back):

    print(front.name)
    print(front.wrench)
    print(back.name)
    print(back.wrench)

def callback_foot(sensor1, sensor2, sensor3):

    print(sensor1.name)
    print(sensor1.wrench)
    print(sensor2.name)
    print(sensor2.wrench)
    print(sensor3.name)
    print(sensor3.wrench)


if __name__ == '__main__':
    try:
        rospy.init_node('wrench_subscriber', anonymous=True)

        fslt_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftThigh/State", RigidBodyState)
        bslt_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftThigh/State", RigidBodyState)
        lt = message_filters.TimeSynchronizer([fslt_sub, bslt_sub], 10)
        lt.registerCallback(callback_leg)

        fsls_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftShank/State", RigidBodyState)
        bsls_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftShank/State", RigidBodyState)
        ls = message_filters.TimeSynchronizer([fsls_sub, bsls_sub], 10)
        ls.registerCallback(callback_leg)

        fsrt_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightThigh/State", RigidBodyState)
        bsrt_sub = message_filters.Subscriber("/ambf/env/BackSensorRightThigh/State", RigidBodyState)
        rt = message_filters.TimeSynchronizer([fsrt_sub, bsrt_sub], 10)
        rt.registerCallback(callback_leg)

        fsrs_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightShank/State", RigidBodyState)
        bsrs_sub = message_filters.Subscriber("/ambf/env/BackSensorRightShank/State", RigidBodyState)
        rs = message_filters.TimeSynchronizer([fsrs_sub, bsrs_sub], 10)
        rs.registerCallback(callback_leg)
       
        lf1_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot1/State", RigidBodyState)
        lf2_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot2/State", RigidBodyState)
        lf3_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot3/State", RigidBodyState)
        lf = message_filters.TimeSynchronizer([lf1_sub, lf2_sub, lf3_sub], 10)
        lf.registerCallback(callback_foot)
        
        rf1_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot1/State", RigidBodyState)
        rf2_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot2/State", RigidBodyState)
        rf3_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot3/State", RigidBodyState)
        rf = message_filters.TimeSynchronizer([rf1_sub, rf2_sub, rf3_sub], 10)
        rf.registerCallback(callback_foot)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
