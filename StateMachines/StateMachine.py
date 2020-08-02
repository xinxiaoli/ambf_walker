#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
from StateMachines.States import Initialize, Stabilize, GMRTest, Follow, Listening, GoTo

class ExoStateMachine(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])

        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'Stabilize'})

            smach.StateMachine.add('Stabilize', Stabilize(model),
                                   transitions={'Stabilizing': 'Stabilize',
                                                'Stabilized': 'outcome4'})

        outcome = sm.execute()




class ExoStateMachineFollowing(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])
        sm.userdata.counter = 0
        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'Listening'}
                                   )

            smach.StateMachine.add('Listening', Listening(model),
                                   transitions={'Waiting': 'Listening',
                                                'Sending': 'Follow'},
                                   remapping={'count': 'count',
                                              'q': 'q'})

            smach.StateMachine.add('Follow', Follow(model),
                                   transitions={'Following': 'Follow',
                                                'Followed': 'Listening'},
                                   remapping={'count': 'count',
                                              'q':'q'})

        outcome = sm.execute()



class ExoStateMachineGoTo(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])
        sm.userdata.counter = 0
        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'GoTo'}
                                   )

            smach.StateMachine.add('GoTo', GoTo(model),
                                   transitions={'Waiting': 'GoTo',
                                                'Sending': 'GoTo'})


        outcome = sm.execute()

class ExoStateMachineTest(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])

        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'Follow'})

            smach.StateMachine.add('Follow', GMRTest(model),
                                   transitions={'Following': 'Follow',
                                                'Followed': 'Follow'})

        outcome = sm.execute()