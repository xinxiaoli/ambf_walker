#!/usr/bin/env python3

import rospy
from Model import Exoskeleton
from ambf_client import Client


_client = Client()
_client.connect()
joints = ['Hip-RobLeftThigh', 'RobLeftThigh-RobLeftShank', 'RobLeftShank-RobLeftFoot', 'Hip-RobRightThigh',
          'RobRightThigh-RobRightShank', 'RobRightShank-RobRightFoot', 'Hip-Crutches']
Exo = Exoskeleton.Exoskeleton(_client, joints, 56, 1.56)

while True:
    left_foot_sensors, right_foot_sensors = Exo.get_foot_sensors()
    print(left_foot_sensors[0])
