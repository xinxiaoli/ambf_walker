#!/usr/bin/env python3

from Model import Exoskeleton
import rospy
from ambf_client import Client

_client = Client()
_client.connect()
rate = rospy.Rate(1000)
joints = ['Hip-RobLeftThigh', 'RobLeftThigh-RobLeftShank', 'RobLeftShank-RobLeftFoot', 'Hip-RobRightThigh',
          'RobRightThigh-RobRightShank', 'RobRightShank-RobRightFoot', 'Hip-Crutches']

Exo = Exoskeleton.Exoskeleton(_client, joints, 69, 1.83)

print(Exo.get_foot_sensors())
