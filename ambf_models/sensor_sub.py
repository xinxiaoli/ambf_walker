#!/usr/bin/env python3

import rospy
import message_filters
from ambf_msgs.msg import RigidBodyState


def callback(fslt,bslt,fsls,bsls,fsrt,bsrt,fsrs,bsrs,lf1,lf2,lf3,rf1,rf2,rf3):

    # Leg segment block, difference in 'force y' signifies direction leg is moving
    print('FrontSensorLeftThigh force y: {}'.format(fslt.wrench.force.y))
    print('BackSensorLeftThigh force y: {}'.format(bslt.wrench.force.y))

    print('FrontSensorLeftShank force y: {}'.format(fsls.wrench.force.y))
    print('BackSensorLeftShank force y: {}'.format(bsls.wrench.force.y))

    print('FrontSensorRightThigh force y: {}'.format(fsrt.wrench.force.y))
    print('BackSensorRightThigh force y: {}'.format(bsrt.wrench.force.y))

    print('FrontSensorRightShank force y: {}'.format(fsrs.wrench.force.y))
    print('BackSensorRightShank force y: {}'.format(bsrs.wrench.force.y))

    # Foot contact block, sign and magnitude of 'force z' signifies contact
    print('SensorLeftFoot1 force z: {}'.format(lf1.wrench.force.z))
    print('SensorLeftFoot2 force z: {}'.format(lf2.wrench.force.z))
    print('SensorLeftFoot3 force z: {}'.format(lf3.wrench.force.z))
    
    print('SensorRightFoot1 force z: {}'.format(rf1.wrench.force.z))
    print('SensorRightFoot2 force z: {}'.format(rf2.wrench.force.z))
    print('SensorRightFoot3 force z: {}'.format(rf3.wrench.force.z))


if __name__ == '__main__':
    try:
        rospy.init_node('wrench_subscriber', anonymous=True)

        fslt_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftThigh/State", RigidBodyState)
        bslt_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftThigh/State", RigidBodyState)

        fsls_sub = message_filters.Subscriber("/ambf/env/FrontSensorLeftShank/State", RigidBodyState)
        bsls_sub = message_filters.Subscriber("/ambf/env/BackSensorLeftShank/State", RigidBodyState)

        fsrt_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightThigh/State", RigidBodyState)
        bsrt_sub = message_filters.Subscriber("/ambf/env/BackSensorRightThigh/State", RigidBodyState)

        fsrs_sub = message_filters.Subscriber("/ambf/env/FrontSensorRightShank/State", RigidBodyState)
        bsrs_sub = message_filters.Subscriber("/ambf/env/BackSensorRightShank/State", RigidBodyState)
       
        lf1_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot1Leg/State", RigidBodyState)
        lf2_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot2Leg/State", RigidBodyState)
        lf3_sub = message_filters.Subscriber("/ambf/env/SensorLeftFoot3Leg/State", RigidBodyState)
        
        rf1_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot1Leg/State", RigidBodyState)
        rf2_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot2Leg/State", RigidBodyState)
        rf3_sub = message_filters.Subscriber("/ambf/env/SensorRightFoot3Leg/State", RigidBodyState)
        
        sub_list = [fslt_sub, bslt_sub, fsls_sub, bsls_sub, fsrt_sub, bsrt_sub, fsrs_sub, bsrs_sub,
                    lf1_sub, lf2_sub, lf3_sub, rf1_sub, rf2_sub, rf3_sub]

        body = message_filters.TimeSynchronizer(sub_list, 1)
        body.registerCallback(callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass  
