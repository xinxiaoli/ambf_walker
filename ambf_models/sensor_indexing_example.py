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
    left_foot_sensor1 = Exo.get_foot_sensors()["lf1"].z
    print(left_foot_sensor1)
