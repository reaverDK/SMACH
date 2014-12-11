#!/usr/bin/env python

__author__ = 'reaver'

import roslib
import rospy
import smach
import smach_ros
import time

from msg.msg import mes_mobile_command, mes_mobile_status

rosnode = " "
destination = " "


# define state wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['STATE_ABORT', 'STATE_NAVIGATE', 'STATE_TIP'])

        self.msg_command = mes_mobile_command()
        self.msg_status = mes_mobile_status()
        self.mobile_publisher = rospy.Publisher('mes_status_topic', mes_mobile_status, queue_size=1)
        rospy.Subscriber('mes_command_topic', mes_mobile_command, self.subcallback)

    def execute(self, userdata):
        rospy.loginfo('Executing state: Wait')
        self.continue_sending = False
        self.outcome = "STATE_WAIT"

        while self.continue_sending:
            self.msg_status.state = self.msg_status.STATE_FREE
            self.msg_status.robot_id = 2
            self.msg_status.version_id = 1
            self.msg_status.done_pct = 1
            self.msg_status.battery = 0
            self.msg_status.position = "FloorOut" #TODO: get current position

            self.msg_status.header.stamp = rospy.Time.now()
            self.mobile_publisher.publish(self.msg_status)
            time.sleep(2)
        return self.outcome

    def subcallback(self, msg):
        if msg.command == self.msg_command.COMMAND_NAVIGATE:
            self.continue_sending = False
            global destination
            destination = msg.path
            self.outcome = "STATE_NAVIGATE"
        if msg.command == self.msg_command.COMMAND_WAIT:
            self.continue_sending = True
        if msg.command == self.msg_command.COMMAND_TIP:
            self.continue_sending = False
            self.outcome = "STATE_TIP"
        if msg.command == self.msg_command.COMMAND_ABORT:
            self.continue_sending = False
            self.outcome = "STATE_ABORT"


# define state navigate
class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['STATE_WAIT'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Navigate')

        cur_position = # get the current position

        if destination == 'FloorIn':
            if cur_position == 'Line':
                # switch to waypoint nav
            elif cur_position == 'FloorOut':
                # continue with waypoint nav
            else:
                #abort and return to Wait
        if destination == 'FloorOut':
            if cur_position == 'RampOut':
                # switch to waypoint nav
            elif cur_position == 'FloorIn':
                # continue with waypoint nav
            else:
                #abort and return to Wait
        if destination == 'Dispenser':
            if cur_position == 'InBox':
                # continue wall Following
            else:
                #abort and return to Wait
        if destination == 'RampIn':
            if cur_position == 'FloorIn':
                # switch to line following
            else:
                #abort and return to Wait
        if destination == 'RampOut':
            if cur_position == 'Dispenser':
                # continue with line following
            elif cur_position == 'InBox':
                #kjhkjg
            else:
                #abort and return to Wait
        if destination == 'InBox':
            if cur_position == 'RampIn':
                # switch to line following
            elif cur_position == 'Station1' or cur_position == 'Station2' or cur_position == 'Station3':
                # Switch to line following/wall following
            else:
                #abort and return to Wait
        if destination == 'Line':
            if cur_position == 'FloorOut':
                # switch to line following
            elif cur_position == 'LoadOff1' or cur_position == 'LoadOff2' or cur_position == 'LoadOff3':
                # switch to line following or something
            elif cur_position == 'LoadOn1' or cur_position == 'LoadOn2' or cur_position == 'LoadOn3':
                # switch to line following or something
            else:
                #abort and return to Wait
        if destination == 'LoadOff1' or destination == 'LoadOff2' or destination == 'LoadOff3':
            if cur_position == 'Line':
                # switch to line following
            else:
                #abort and return to Wait
        if destination == 'LoadOn1' or destination == 'LoadOn2' or destination == 'LoadOn3':
            if cur_position == 'Line':
                # switch to line following
            else:
                #abort and return to Wait
        if destination == 'Station1' or destination == 'Station2' or destination == 'Station3':
            if cur_position == 'InBox':
                # switch to line following
            else:
                #abort and return to Wait

        return 'STATE_WAIT'


# define state tip
class Tip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['STATE_WAIT'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Tip')

        #Activate Tipper

        return 'STATE_WAIT'


# define state abort
class Abort(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['STATE_WAIT'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Abort')

        #display error or warning msg and return to wait if possible

        return 'STATE_WAIT'


#main
def main():
    global rosnode
    rosnode = rospy.init_node('SMACH_in_da_house')

    # creates a smach state machine
    sm = smach.StateMachine(outcomes=[])

    # opening the container
    with sm:
        # add states to the container
        smach.StateMachine.add('Navigate', Navigate(),
                               transitions={'STATE_WAIT': 'Wait',
                                            'STATE_ABORT': 'Abort'})
        smach.StateMachine.add('Wait', Wait(),
                               transitions={'STATE_NAVIGATE': 'Navigate',
                                            'STATE_TIP': 'Tip',
                                            'STATE_ABORT': 'Abort'})
        smach.StateMachine.add('Tip', Tip(),
                               transitions={'STATE_WAIT': 'Wait',
                                            'STATE_ABORT': 'Abort'})
        smach.StateMachine.add('Abort', Abort(),
                               transitions={'STATE_WAIT': 'Wait'})

    # set the initial state
    sm.set_initial_state([Wait])

    # execute smach plan
    outcome = sm.execute()

    rospy.loginfo(outcome)

    rospy.spin()

if __name__ == '__main__':
    main()