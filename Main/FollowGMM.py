#!/usr/bin/env python
import sys
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from ambf_walker.msg import DesiredJoints
from StateMachines import StateMachine
from Controller import ControllerNode
from Model import Exoskeleton
import rospy
from ambf_client import Client


_client = Client()
_client.connect()
rate = rospy.Rate(1000)
LARRE = Exoskeleton.Exoskeleton(_client, 56, 1.56)
cnrl = ControllerNode.ControllerNode(LARRE)
machine = StateMachine.ExoStateMachineTest(LARRE)

