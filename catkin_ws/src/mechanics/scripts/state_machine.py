#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs import msg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from collections import OrderedDict
import typing

class ServoBridge:
    def __init__(self, 
                 topic_name: str,
                 servos_dict: typing.Dict[str, typing.Dict[str, int]], 
                 default_angles: typing.List[int] = None,
                 timer_hz: float = None
                 ) -> None:
        self._servo_publisher = rospy.Publisher(topic_name, msg.Float32MultiArray, queue_size=1)

        self._servos_dict = OrderedDict(servos_dict)
        self._servos_angles = default_angles

        if timer_hz != None:
            rospy.Timer(rospy.Duration(1 / timer_hz), self._publish)

        if default_angles == None:
            rospy.logwarn(f'default_angles not stated by user')
            
            self._servos_angles = [list(servo_states.values())[0] for servo_states in self._servos_dict.values()]

        elif len(default_angles) != len(servos_dict):
            rospy.logwarn(f'default_angles len should be {len(servos_dict)}. Now is {len(default_angles)}')

            self._servos_angles = [servo_states.values()[0] for servo_states in self._servos_dict.values()]

        self._publish()

    def set(self, states_dict: typing.Dict[str, str]) -> bool:
        return_flag = False

        for servo_name, state_name in states_dict.items():
            servo_states = self._servos_dict.get(servo_name, None)

            if servo_states == None:
                rospy.logerr(f'No such servo name: {servo_name}')
                return_flag = True
                continue

            servo_angle = servo_states.get(state_name, None)

            if servo_angle == None:
                rospy.logerr(f'No such servo state for {servo_name}: {state_name}')
                return_flag = True
                continue

            self._servos_angles[list(self._servos_dict.keys()).index(servo_name)] = servo_angle

        self._publish()

        return return_flag

    def _publish(self, event: rospy.timer.TimerEvent = None):
        self._servo_publisher.publish(msg.Float32MultiArray(data=self._servos_angles))


class Servos(smach.State):
    def __init__(self, 
                 servo_bridge: ServoBridge,
                 state_dict: typing.Dict[str, str],
                 sleep_duration: float = None
                 ) -> None:
        smach.State.__init__(self, outcomes=['succeed', 'aborted'])
        
        self.servo_bridge = servo_bridge
        self.state_dict = state_dict
        self.sleep_duration = sleep_duration

    def execute(self, userdata: smach.UserData):
        flag = self.servo_bridge.set(self.state_dict)

        if self.sleep_duration != None:
            rospy.sleep(self.sleep_duration)

        if flag:
            return 'aborted'
        else:
            return 'succeed'


class SleepState(smach.State):
    def __init__(self, duration: float) -> None:
        smach.State.__init__(self, outcomes=['succeed'])
        
        self.duration = duration

    def execute(self, userdata: smach.UserData):
        rospy.sleep(self.duration)

        return 'succeed'


class StartWaiter(smach.State):
    def __init__(self, topic_name):
        smach.State.__init__(self, outcomes=['succeed'])

        self.is_started = False

        rospy.Subscriber(topic_name, msg.Bool, self.callback)

    def callback(self, status: msg.Bool):
        if status.data:
            self.is_started = True
    
    def execute(self, userdata: smach.UserData):
        while not self.is_started:
            rospy.sleep(0.05)

        return 'succeed'


def get_push_cakes_sm(servo_bridge):
    push_cake_sm = smach.StateMachine(outcomes=['succeed', 'aborted'])

    with push_cake_sm:
        smach.StateMachine.add(
            'Open Gripper',
            Servos
                (
                    servo_bridge,
                    {'left_gripper' : 'opened', 'right_gripper' : 'opened', 'lift' : 'down'},
                    sleep_duration=3
                ),
            transitions = {'succeed' : 'Close Gripper', 'aborted' : 'aborted'})
                
        smach.StateMachine.add(
            'Close Gripper',
            Servos
                (
                    servo_bridge,
                    {'left_gripper' : 'closed', 'right_gripper' : 'closed'},
                    sleep_duration=0.5
                ),
            transitions = {'succeed' : 'Lift Up', 'aborted' : 'aborted'})
        
        smach.StateMachine.add(
            'Lift Up',
            Servos
                (
                    servo_bridge,
                    {'lift' : 'up'},
                    sleep_duration=3
                ),
            transitions = {'succeed' : 'Plunger Out', 'aborted' : 'aborted'})
        
        # smach.StateMachine.add(
        #     'Plunger Out',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'plunger' : 'out'},
        #             sleep_duration=0.5
        #         ),
        #     transitions = {'succeed' : 'succeed', 'aborted' : 'aborted'})



def main():
    rospy.init_node('state_machine')


    servo_bridge = ServoBridge(
        "servo_cmd_topic",
        {
            'left_gripper'      :   {'opened' : 130,    'closed': 60,   'preclosed': 65},
            'right_gripper'     :   {'opened' : 55,     'closed': 125,  'preclosed': 120},
            'lift'              :   {'down' : 15,       'up' : 200},
            'plunger'           :   {'in' : 95,        'out' : 150},

            'left_limiter'      :   {'on' : 55,         'off' : 105},
            'right_limiter'     :   {'on' : 230,        'off' : 180},
            'cherry_spreader'   :   {'down' : 10,       'up' : 285},
            'cherry_separator'  :   {'left' : 115,      'middle' : 210, 'right' : 240},

            'visor'             :   {'up' : 10,          'down' : 235}
        },
        timer_hz=20
    )

    state_machine = smach.StateMachine(outcomes=['succeed', 'aborted'])

    with state_machine:
        # smach.StateMachine.add(
        #     'Sleep_5', 
        #     SleepState(5),
        #     transitions={'succeed': 'Init State'})

        smach.StateMachine.add(
            'Init State', 
            Servos
                (
                    servo_bridge,
                        {
                            'left_gripper' : 'opened', 
                            'right_gripper' : 'opened', 
                            'lift' : 'down',
                            'plunger' : 'in',

                            'left_limiter' : 'off',
                            'right_limiter' : 'off',
                            'cherry_spreader' : 'down',
                            'cherry_separator' : 'middle',

                            'visor' : 'down'

                        },
                    sleep_duration=5
                ),

            transitions = {'succeed' : 'Waiting start', 'aborted' : 'aborted'})


        # smach.StateMachine.add(
        #     f'Lift Down', 
        #     Servos(servo_bridge, {'cherry_spreader' : 'down'}, sleep_duration=2),
        #     transitions={'succeed': 'Middle 1', 'aborted':'aborted'})

        # for i in range(1, 11):
        #     smach.StateMachine.add(
        #         f'Middle {i}', 
        #         Servos(servo_bridge, {'cherry_separator' : 'middle'}, sleep_duration=1),
        #         transitions={'succeed': f'Left {i}', 'aborted':'aborted'})
            
        #     smach.StateMachine.add(
        #         f'Left {i}', 
        #         Servos(servo_bridge, {'cherry_separator' : 'left'}, sleep_duration=3),
        #         transitions={'succeed': f'Middle {i+1}', 'aborted':'aborted'})
            
        # smach.StateMachine.add(
        #     'Middle 11', 
        #     Servos(servo_bridge, {'cherry_separator' : 'middle'}, sleep_duration=1),
        #     transitions={'succeed': 'succeed', 'aborted':'aborted'})

        # smach.StateMachine.add(
        #     'Push Cake',
        #     get_push_cakes_sm(servo_bridge),
        #     transitions = {'succeed' : 'succeed', 'aborted' : 'aborted'})

        smach.StateMachine.add(
            'Waiting start',
            StartWaiter('start_topic'),
            transitions = {'succeed' : 'MOVE'}
        )

        # smach.StateMachine.add(
        #     'Open Gripper',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'left_gripper' : 'opened', 'right_gripper' : 'opened', 'lift' : 'down'},
        #             sleep_duration=3
        #         ),
        #     transitions = {'succeed' : 'Close Gripper', 'aborted' : 'aborted'})

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0.55
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add (
            'MOVE', 
            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=goal),
            transitions={'succeeded' : 'Close Gripper', 'preempted' : 'aborted', 'aborted' : 'aborted'})
                
        smach.StateMachine.add(
            'Close Gripper',
            Servos
                (
                    servo_bridge,
                    {'left_gripper' : 'closed', 'right_gripper' : 'closed'},
                    sleep_duration=1
                ),
            transitions = {'succeed' : 'MOVE_1', 'aborted' : 'aborted'})
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w = 1.0
        smach.StateMachine.add (
            'MOVE_1', 
            smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=goal),
            transitions={'succeeded' : 'succeed', 'preempted' : 'aborted', 'aborted' : 'aborted'})
        
        
        # smach.StateMachine.add(
        #     'PreClose Gripper',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'left_gripper' : 'preclosed', 'right_gripper' : 'preclosed'},
        #             sleep_duration=1
        #         ),
        #     transitions = {'succeed' : 'Lift Up', 'aborted' : 'aborted'})
        
        # smach.StateMachine.add(
        #     'Lift Up',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'lift' : 'up'},
        #             sleep_duration=3
        #         ),
        #     transitions = {'succeed' : 'Left Limiter On', 'aborted' : 'aborted'})
        
        # smach.StateMachine.add(
        #     'Left Limiter On',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'left_limiter' : 'on', 'right_limiter' : 'off', 'visor' : 'down'},
        #             sleep_duration=1
        #         ),
        #     transitions = {'succeed' : 'Plunger In', 'aborted' : 'aborted'})
        
        # smach.StateMachine.add(
        #     'Plunger In',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'plunger' : 'in'},
        #             sleep_duration=3
        #         ),
        #     transitions = {'succeed' : 'Plunger Out', 'aborted' : 'aborted'})
        
        # smach.StateMachine.add(
        #     'Plunger Out',
        #     Servos
        #         (
        #             servo_bridge,
        #             {'plunger' : 'out'},
        #             sleep_duration=3
        #         ),
        #     transitions = {'succeed' : 'succeed', 'aborted' : 'aborted'})
            

    sis = smach_ros.IntrospectionServer('server', state_machine, 'SM_ROOT')
    sis.start()

    outcome = state_machine.execute()
    # outcome = push_cake_sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()