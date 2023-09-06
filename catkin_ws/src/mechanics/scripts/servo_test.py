#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
import dynamic_reconfigure.parameter_generator as pg
# from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    pass
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\ 
    #       {str_param}, {bool_param}, {size}""".format(**config))
    # return config

if __name__ == "__main__":
    node_name = 'servo_test'
    rospy.init_node(node_name)

    test_var = rospy.get_param(node_name + '/test')

    rospy.logwarn(test_var)


    gen = pg.ParameterGenerator()
    #       Name       Type      Level Description     Default Min   Max
    gen.add("message", pg.str_t,    0,    "The message.", "hello")
    # gen.add("a",       int_t,    0,    "First number.", 1,     -100, 100)
    # gen.add("b",       int_t,    0,    "First number.", 2,     -100, 100)

    srv = Server(gen., callback)
    srv = Server()
    # Server.update_configuration()
    rospy.spin()