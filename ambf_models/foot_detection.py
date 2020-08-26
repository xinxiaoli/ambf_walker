#!/usr/bin/env python3

import rospy
import numpy as np
import message_filters
from ambf_msgs.msg import RigidBodyState

packet = 0
lf1_z =  []
lf2_z =  []
lf3_z =  []
rf1_z =  []
rf2_z =  []
rf3_z =  []


def callback(lf1,lf2,lf3,rf1,rf2,rf3):

    global packet
    global lf1_z
    global lf2_z
    global lf3_z
    global rf1_z
    global rf2_z
    global rf3_z

    lf1_z = np.append(lf1_z, np.absolute(lf1.wrench.force.z))
    lf2_z = np.append(lf2_z, np.absolute(lf2.wrench.force.z))
    lf3_z = np.append(lf3_z, np.absolute(lf3.wrench.force.z))
    rf1_z = np.append(rf1_z, np.absolute(rf1.wrench.force.z))
    rf2_z = np.append(rf2_z, np.absolute(rf2.wrench.force.z))
    rf3_z = np.append(rf3_z, np.absolute(rf3.wrench.force.z))

    packet += 1

    if packet == 10:
        LF1 = np.average(lf1_z)
        LF2 = np.average(lf2_z)
        LF3 = np.average(lf3_z)
        LF = np.average([LF1, LF2, LF3])
        RF1 = np.average(rf1_z)
        RF2 = np.average(rf2_z)
        RF3 = np.average(rf3_z)
        RF = np.average([RF1, RF2, RF3])

        if np.absolute(LF) > 10:
            print('LeftFoot average force z: {}, Contact'.format(LF))
        else:
            print('LeftFoot average force z: {}, No contact'.format(LF))

        if np.absolute(RF) > 10:
            print('RightFoot average force z: {}, Contact'.format(RF))
        else:
            print('RightFoot average force z: {}, No contact'.format(RF))

        packet = 0
        lf1_z =  []
        lf2_z =  []
        lf3_z =  []
        rf1_z =  []
        rf2_z =  []
        rf3_z =  []


if __name__ == '__main__':


    try:
        rospy.init_node('foot_detection', anonymous=True)
       
        lf1_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot1Leg/State", RigidBodyState)
        lf2_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot2Leg/State", RigidBodyState)
        lf3_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot3Leg/State", RigidBodyState)
        
        rf1_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot1Leg/State", RigidBodyState)
        rf2_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot2Leg/State", RigidBodyState)
        rf3_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot3Leg/State", RigidBodyState)
        
        sub_list = [lf1_sub, lf2_sub, lf3_sub, rf1_sub, rf2_sub, rf3_sub]

        body = message_filters.TimeSynchronizer(sub_list, 10)
        body.registerCallback(callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
