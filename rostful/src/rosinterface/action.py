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

from . import message_conversion as msgconv
from . import deffile, definitions

from .util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse

"""
ActionBack is the class handling conversion from REST API to ROS Action

Publications:
 * /averaging/status [actionlib_msgs/GoalStatusArray]
 * /averaging/result [actionlib_tutorials/AveragingActionResult]
 * /rosout [rosgraph_msgs/Log]
 * /averaging/feedback [actionlib_tutorials/AveragingActionFeedback]

Subscriptions:
 * /random_number [unknown type]
 * /averaging/goal [actionlib_tutorials/AveragingActionGoal]
 * /averaging/cancel [actionlib_msgs/GoalID]
 """
class ActionBack:
    STATUS_SUFFIX = 'status'
    RESULT_SUFFIX = 'result'
    FEEDBACK_SUFFIX = 'feedback'
    GOAL_SUFFIX = 'goal'
    CANCEL_SUFFIX = 'cancel'

    def __init__(self, action_name, action_type, queue_size=1):
        self.name = action_name
        # getting the fullname to make sure we start with /
        self.fullname = self.name if self.name.startswith('/') else '/' + self.name

        action_type_module, action_type_name = tuple(action_type.split('/'))
        roslib.load_manifest(action_type_module)
        msg_module = import_module(action_type_module + '.msg')

        self.rostype_name = action_type

        self.rostype_action = getattr(msg_module, action_type_name + 'Action')

        self.rostype_action_goal = getattr(msg_module, action_type_name + 'ActionGoal')
        self.rostype_action_result = getattr(msg_module, action_type_name + 'ActionResult')
        self.rostype_action_feedback = getattr(msg_module, action_type_name + 'ActionFeedback')

        self.rostype_goal = getattr(msg_module, action_type_name + 'Goal')
        self.rostype_result = getattr(msg_module, action_type_name + 'Result')
        self.rostype_feedback = getattr(msg_module, action_type_name + 'Feedback')

        self.actiontype = definitions.get_action_action_dict(self)
        self.status_msg = deque([], queue_size)
        self.status_sub = rospy.Subscriber(self.name + '/' +self.STATUS_SUFFIX, actionlib_msgs.msg.GoalStatusArray, self.status_callback)

        self.result_msg = deque([], queue_size)
        self.result_sub = rospy.Subscriber(self.name + '/' + self.RESULT_SUFFIX, self.rostype_action_result, self.result_callback)

        self.feedback_msg = deque([], queue_size)
        self.feedback_sub = rospy.Subscriber(self.name + '/' +self.FEEDBACK_SUFFIX, self.rostype_action_feedback, self.feedback_callback)

        self.goal_pub = rospy.Publisher(self.name + '/' + self.GOAL_SUFFIX, self.rostype_action_goal, queue_size=1)
        self.cancel_pub = rospy.Publisher(self.name + '/' +self.CANCEL_SUFFIX, actionlib_msgs.msg.GoalID, queue_size=1)

    def get_msg_type(self, suffix):
        if suffix == self.STATUS_SUFFIX:
            return actionlib_msgs.msg.GoalStatusArray
        elif suffix == self.RESULT_SUFFIX:
            return self.rostype_action_result
        elif suffix == self.FEEDBACK_SUFFIX:
            return self.rostype_action_feedback
        elif suffix == self.GOAL_SUFFIX:
            return self.rostype_action_goal
        elif suffix == self.CANCEL_SUFFIX:
            return actionlib_msgs.msg.GoalID
        else:
            return None

    def publish_goal(self, msg):
        rospy.logwarn('PUBLISH_GOAL')
        self.goal_pub.publish(msg)
        return

    def publish_cancel(self, msg):
        self.cancel_pub.publish(msg)
        return

    def publish(self, suffix, msg):
        rospy.logwarn('publish %r %r', suffix, msg)
        if suffix == self.GOAL_SUFFIX:
            self.publish_goal(msg)
        elif suffix == self.CANCEL_SUFFIX:
            self.publish_cancel(msg)

    def get_status(self, num=None):
        if not self.status_msg:
            return None

        return self.status_msg[0]

    def get_result(self, num=None):
        if not self.result_msg:
            return None

        return self.result_msg[0]

    def get_feedback(self, num=None):
        if not self.feedback_msg:
            return None

        return self.feedback_msg[0]

    def get(self, suffix, num=None):
        if suffix == self.STATUS_SUFFIX:
            return self.get_status(num=num)
        elif suffix == self.RESULT_SUFFIX:
            return self.get_result(num=num)
        elif suffix == self.FEEDBACK_SUFFIX:
            return self.get_feedback(num=num)
        else:
            return None

    def status_callback(self, msg):
        self.status_msg.appendleft(msg)

    def result_callback(self, msg):
        self.result_msg.appendleft(msg)

    def feedback_callback(self, msg):
        self.feedback_msg.appendleft(msg)
