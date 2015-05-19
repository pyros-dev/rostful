from __future__ import absolute_import

import roslib
import rospy
from rospy.service import ServiceManager
import rosservice, rostopic
import actionlib_msgs.msg

from importlib import import_module
from collections import deque

import json
import sys
import re
from StringIO import StringIO

from .ros_interface import RosInterface
from .rocon_interface import RoconInterface

from dynamic_reconfigure.server import Server
from rostful.cfg import RostfulConfig
import ast

"""
Interface with ROS.
No inheritance to make sure destructor is called properly.
"""
class RosNode():
    def __init__(self, ros_args):
        #we initialize the node here, passing ros parameters
        rospy.init_node('rostful_server', argv=ros_args, anonymous=True, disable_signals=True)
        rospy.logwarn('rostful_server node started with args : %r', ros_args)

        self.ros_if = RosInterface()
        self.rocon_if = RoconInterface(self.ros_if)

        # Create a dynamic reconfigure server.
        self.server = Server(RostfulConfig, self.reconfigure)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        rospy.logwarn("""Reconfigure Request coming in rostful_node!""")
        config = self.ros_if.reconfigure(config, level)
        config = self.rocon_if.reconfigure(config, level)
        return config

    @property
    def ros_if(self):
        return self.ros_if

    @property
    def rocon_if(self):
        return self.rocon_if


    def spin(self):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # do stuff here only if needed in emergency.
            # Async event based programming is preferred.
            rate.sleep()  # loop timer only here
        rospy.spin()

    """
    Does all necessary cleanup to bring down RosInterface
    """
    def __del__(self):
        rospy.logwarn('rostful_server node stopped')
        rospy.signal_shutdown('Closing')


