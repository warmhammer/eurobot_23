#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs import msg

from collections import OrderedDict
import typing

class ServoBridge:
    def __init__(self, 
                 topic_name: str,
                 servos_dict: typing.Dict[str, typing.Dict[str, int]], 
                 default_angles: typing.List[int] = None,
                 timer_hz: float = None
                 ) -> None:
        self._servo_publisher = rospy.Publisher(topic_name, msg.Float32MultiArray)

        self._servos_dict = OrderedDict(servos_dict)
        self._servos_angles = default_angles

        if timer_hz != None:
            rospy.Timer(rospy.Duration(1 / timer_hz), self._publish)

        if default_angles == None:
            rospy.loginfo(f'default_angles not stated by user')
            
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
        rospy.logwarn(f'{self._servos_angles}')
        self._servo_publisher.publish(msg.Float32MultiArray(data=self._servos_angles))


# class Initialization(smach.State):
#     def __init__(self, 
#                  servo_bridge_set: ServoBridge.set,
#                  state_dict: typing.Dict[str, str]
#                  ) -> None:
#         smach.State.__init__(self, outcomes=['succeed', 'aborted'])
        
#         self.servo_bridge_set = servo_bridge_set
#         self.state_dict = state_dict

#     def execute(self, userdata: smach.UserData):
#         rospy.logwarn(f'servo')

#         flag = self.servo_bridge_set(self.state_dict)

#         if flag:
#             return 'aborted'
#         else:
#             return 'succeed'


class Servos(smach.State):
    def __init__(self, 
                 servo_bridge_set: ServoBridge.set,
                 state_dict: typing.Dict[str, str]
                 ) -> None:
        smach.State.__init__(self, outcomes=['succeed', 'aborted'])
        
        self.servo_bridge_set = servo_bridge_set
        self.state_dict = state_dict

    def execute(self, userdata: smach.UserData):
        rospy.logwarn(f'servo')

        flag = self.servo_bridge_set(self.state_dict)

        if flag:
            return 'aborted'
        else:
            return 'succeed'


class SleepState(smach.State):
    def __init__(self, duration: float) -> None:
        smach.State.__init__(self, outcomes=['succeed'])
        
        self.duration = duration

    def execute(self, userdata: smach.UserData):
        rospy.logwarn(f'Waiting {self.duration}')

        rospy.sleep(self.duration)

        return 'succeed'


def callback():
    pass

# main
def main():
    rospy.init_node('state_machine')

    sub = rospy.Subscriber('/servo_cmd_topic', msg.Float32MultiArray, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
