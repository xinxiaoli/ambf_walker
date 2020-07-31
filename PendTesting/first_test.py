#!/usr/bin/env python
import sys
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))


from ambf_walker.msg import DesiredJoints
from Model import DoublePendulm
import rospy
from ambf_client import Client


_client = Client()
_client.connect()
rate = rospy.Rate(1000)
pend = DoublePendulm.DoublePendulum(_client)
while 1:
    pass
