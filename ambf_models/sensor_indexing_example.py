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
    left_leg_sensors, right_leg_sensors = Exo.get_leg_sensors()
    print(left_leg_sensors[0])
