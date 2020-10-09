#!/usr/bin/env python
import smach
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from ambf_walker.msg import DesiredJoints
from StateMachines.States import *

class ExoStateMachine(object):

    def __init__(self, model):
        sm = smach.StateMachine(outcomes=['outcome4'])

        with sm:
            smach.StateMachine.add('Initialize', Initialize(model=model),
                                    transitions={'Initializing': 'Initialize',
                                                  'Initialized': 'Main'})

            smach.StateMachine.add('Main', Main(model, ["Poly", "DMP", "Lower", "MPC2", "LQR", "Temp", "stairDMP", "walk" ] ),
                                   transitions={'Poly': 'Listening',
                                                'DMP': 'DMP',
                                                'Lower':'LowerBody',
                                                'MPC2':'MPC2',
                                                "LQR":"LQR",
                                                "Temp":"Temp",
                                                "stairDMP":"stairDMP",
                                                "walk": "walk"})

            smach.StateMachine.add('LowerBody', LowerBody(model),
                                   transitions={'Lowering': 'LowerBody',
                                                'Lowered': 'Main'})

            smach.StateMachine.add('DMP', DMP(model),
                                   transitions={'stepping': 'DMP',
                                                'stepped': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('MPC2', MPC2(model),
                                   transitions={'MPC2ing': 'MPC2',
                                                'MPC2ed': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('LQR', LQR(model),
                                   transitions={'LQRing': 'LQR',
                                                'LQRed': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Listening', Listening(model),
                                   transitions={'Waiting': 'Listening',
                                                'Sending': 'Follow'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('Follow', Follow(model),
                                   transitions={'Following': 'Follow',
                                                'Followed': 'Main'},
                                   remapping={'q': 'q'})


            smach.StateMachine.add('stairDMP', StairDMP(model),
                                   transitions={'stairing': 'stairDMP',
                                                'staired': 'Main'},
                                   remapping={'q': 'q'})

            smach.StateMachine.add('walk', Walk(model),
                                   transitions={'walking': 'walk',
                                                'walked': 'Main'},
                                   remapping={'q': 'q'})

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