import smach
import rospy


class Initialize(smach.State):

    def __init__(self, outcomes=['Initializing', 'Initialized']):

        smach.State.__init__(self, outcomes=outcomes)
        self.count = 0

    def execute(self, userdata):

        if self.count <= 500:
            self.count += 1
            print "Initialize ", self.count
            return "Initializing"
        else:
            self.count = 0
            return "Initialized"


class Stabilze(smach.State):

    def __init__(self, outcomes=['Stabilzing', 'Stabilzed']):
        smach.State.__init__(self, outcomes=outcomes)
        self.count = 0

    def execute(self, userdata):

        if self.count <= 500:
            self.count += 1
            print "stab ", self.count
            return "Stabilzing"
        else:
            self.count = 0
            return "Stabilzed"



def main():


    sm = smach.StateMachine(outcomes=['outcome5'])

    with sm:
        smach.StateMachine.add('Initialize', Initialize(),
                               transitions={'Initializing': 'Initialize',
                                            'Initialized': 'Stabilze'})
        smach.StateMachine.add('Stabilze', Stabilze(),
                               transitions={'Stabilzing': 'Stabilze',
                                            'Stabilzed': 'Initialize'})

    outcome = sm.execute()


if __name__  == '__main__':
    main()
