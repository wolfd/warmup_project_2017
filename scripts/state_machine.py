#!/usr/bin/env python2

import rospy
import smach
import smach_ros

from square_dance import SquareDance
from obstacle_avoider import ObstacleAvoider

def main():
    rospy.init_node('square_and_avoid_state_machine')

    # create a smach state machine
    sm = smach.StateMachine(outcomes=['give_up'])
    with sm:
        smach.StateMachine.add(
            'obstacle_avoider',
            ObstacleAvoider(),
            transitions={ # define the transitions that obstacle_avoider can go through to other states
                'reached_goal': 'square_dance',
                'timed_out': 'give_up'
            }
        )

        smach.StateMachine.add(
            'square_dance',
            SquareDance(),
            transitions={
                'reached_end': 'obstacle_avoider',
                'hit_something': 'give_up'
            }
        )

    # start the state machine
    outcome = sm.execute()


if __name__ == '__main__':
    main()
