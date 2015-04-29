from __future__ import absolute_import

import roslib
roslib.load_manifest('rostful')
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
ServiceBack is the class handling conversion from REST API to ROS Service
"""
class ServiceBack:
    def __init__(self, service_name, service_type):
        self.name = service_name

        service_type_module, service_type_name = tuple(service_type.split('/'))
        roslib.load_manifest(service_type_module)
        srv_module = import_module(service_type_module + '.srv')

        self.rostype_name = service_type
        self.rostype = getattr(srv_module, service_type_name)
        self.rostype_req = getattr(srv_module, service_type_name + 'Request')
        self.rostype_resp = getattr(srv_module, service_type_name + 'Response')

        self.srvtype = definitions.get_service_srv_dict(self)
        rospy.logwarn('srvtype : %r', self.srvtype)

        self.proxy = rospy.ServiceProxy(self.name, self.rostype)

    def call(self, rosreq):
#       rosreq = self.rostype_req()
#       if use_ros:
#           rosreq.deserialize(req)
#       else:
#           msgconv.populate_instance(req, rosreq)

        fields = []
        for slot in rosreq.__slots__:
            fields.append(getattr(rosreq, slot))
        fields = tuple(fields)

        return self.proxy(*fields)

"""
TopicBack is the class handling conversion from REST API to ROS Topic
"""
class TopicBack:
    def __init__(self, topic_name, topic_type, allow_pub=True, allow_sub=True, queue_size=1):
        self.name = topic_name

        topic_type_module, topic_type_name = tuple(topic_type.split('/'))
        roslib.load_manifest(topic_type_module)
        msg_module = import_module(topic_type_module + '.msg')

        self.rostype_name = topic_type
        self.rostype = getattr(msg_module, topic_type_name)

        self.allow_pub = allow_pub
        self.allow_sub = allow_sub

        self.msgtype = definitions.get_topic_msg_dict(self)
        self.msg = deque([], queue_size)

        self.pub = None
        if self.allow_pub:
            self.pub = rospy.Publisher(self.name, self.rostype, queue_size=1 )

        self.sub = None
        if self.allow_sub:
            self.sub = rospy.Subscriber(self.name, self.rostype, self.topic_callback)

    def publish(self, msg):
        self.pub.publish(msg)
        return

    def get(self, num=None):
        if not self.msg:
            return None

        return self.msg[0]

    def topic_callback(self, msg):
        self.msg.appendleft(msg)

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


CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'
ACTION_PATH = '_action'

def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH,SRV_PATH,MSG_PATH,ACTION_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

def response(start_response, status, data, content_type):
    content_length = 0
    if data is not None:
        content_length = len(data)
    headers = [('Content-Type', content_type), ('Content-Length', str(content_length))]
    start_response(status, headers)
    return data

def response_200(start_response, data='', content_type='application/json'):
    return response(start_response, '200 OK', data, content_type)

def response_404(start_response, data='Invalid URL!', content_type='text/plain'):
    return response(start_response, '404 Not Found', data, content_type)

def response_405(start_response, data=[], content_type='text/plain'):
    return response(start_response, '405 Method Not Allowed', data, content_type)

def response_500(start_response, error, content_type='text/plain'):
    e_str = '%s: %s' % (str(type(error)), str(error))
    return response(start_response, '500 Internal Server Error', e_str, content_type)


from dynamic_reconfigure.server import Server
from rostful.cfg import RostfulConfig
import ast

"""
Interface with ROS
"""
class ROSIF():
    def __init__(self):
        #we initialize the node here
        rospy.init_node('rostful_server', anonymous=True, disable_signals=True)

        #current services topics and actions exposed
        self.services = {}
        self.topics = {}
        self.actions = {}
        #last requested services topics and actions to be exposed
        self.services_args = {}
        self.topics_args = {}
        self.actions_args = {}

        topics_args = ast.literal_eval(rospy.get_param('~topics', "[]"))
        services_args = ast.literal_eval(rospy.get_param('~services', "[]"))
        actions_args = ast.literal_eval(rospy.get_param('~actions', "[]"))

        rospy.logwarn('Init topics : %s', topics_args)
        self.expose_topics(topics_args)
        rospy.logwarn('Init services : %s', services_args)
        self.expose_services(services_args)
        rospy.logwarn('Init actions : %s', actions_args)
        self.expose_actions(actions_args)

        self.topics_args = topics_args
        self.services_args = services_args
        self.actions_args = actions_args

        # Create a dynamic reconfigure server.
        self.server = Server(RostfulConfig, self.reconfigure)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):

        rospy.logwarn("""Reconfigure Request: \ntopics : {topics} \nservices : {services} \nactions : {actions}""".format(**config))
        new_topics = ast.literal_eval(config["topics"])
        rospy.logwarn('%r',new_topics)
        self.expose_topics(new_topics)
        new_services = ast.literal_eval(config["services"])
        self.expose_services(new_services)
        new_actions = ast.literal_eval(config["actions"])
        self.expose_actions(new_actions)

        return config

    def add_service(self, service_name, ws_name=None, service_type=None):
        resolved_service_name = rospy.resolve_name(service_name)
        if service_type is None:
            service_type = rosservice.get_service_type(resolved_service_name)
            if not service_type:
                rospy.logwarn('Unknown service %s' % service_name)
                return False

        if ws_name is None:
            ws_name = service_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.services[ws_name] = ServiceBack(service_name, service_type)
        return True

    def del_service(self, service_name, ws_name=None):
        if ws_name is None:
            ws_name = service_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.services.pop(ws_name,None)
        return True

    """
    This exposes a list of services as REST API. services not listed here will be removed from the API
    """
    def expose_services(self, service_names):
        rospy.logwarn('Exposing services : %r', service_names)
        if not service_names:
            return
        for service_name in service_names:
            if not service_name in self.services_args:
                ret = self.add_service(service_name)
                if ret: rospy.loginfo( 'Added Service %s', service_name )

        for service_name in self.services_args:
            if not service_name in service_names:
                ret = self.del_service(service_name)
                if ret: rospy.loginfo ( 'Removed Service %s', service_name )

        #Updating the list of services
        self.services_args = service_names

    def add_topic(self, topic_name, ws_name=None, topic_type=None, allow_pub=True, allow_sub=True):
        resolved_topic_name = rospy.resolve_name(topic_name)
        if topic_type is None:
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if not topic_type:
                rospy.loginfo( 'Unknown topic %s' % topic_name )
                return False

        if ws_name is None:
            ws_name = topic_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.topics[ws_name] = TopicBack(topic_name, topic_type, allow_pub=allow_pub, allow_sub=allow_sub)
        return True

    def del_topic(self, topic_name, ws_name=None):
        if ws_name is None:
            ws_name = topic_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.topics.pop(ws_name,None)
        return True

    """
    This exposes a list of topics as REST API. topics not listed here will be removed from the API
    """
    def expose_topics(self, topic_names, allow_pub=True, allow_sub=True):
        rospy.logwarn('Exposing topics : %r', topic_names)
        if not topic_names:
            return
        # Adding missing ones
        for topic_name in topic_names:
            if not topic_name in self.topics_args:
                ret = self.add_topic(topic_name, allow_pub=allow_pub, allow_sub=allow_sub)
                if ret: rospy.loginfo ( 'Exposing Topic %s Pub %r Sub %r', topic_name, allow_pub, allow_sub)

        # Removing extra ones
        for topic_name in self.topics_args:
            if not topic_name in topic_names:
                ret = self.del_topic(topic_name)
                if ret: rospy.loginfo ( 'Removing Topic %s', topic_name)

        # Updating the list of topics
        self.topics_args = topic_names

    def add_action(self, action_name, ws_name=None, action_type=None):
        if action_type is None:
            resolved_topic_name = rospy.resolve_name(action_name + '/result')
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if not topic_type:
                rospy.logwarn( 'Unknown action %s', action_name )
                return False
            action_type = topic_type[:-len('ActionResult')]

        if ws_name is None:
            ws_name = action_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.actions[ws_name] = ActionBack(action_name, action_type)
        return True

    def del_action(self, action_name, ws_name=None):
        if ws_name is None:
            ws_name = action_name
        if ws_name.startswith('/'):
            ws_name = ws_name[1:]

        self.actions.pop(ws_name,None)
        return True

    """
    This exposes a list of actions as REST API. actions not listed here will be removed from the API
    """
    def expose_actions(self, action_names):
        rospy.logwarn('Exposing actions : %r', action_names)
        if not action_names:
            return
        for action_name in action_names:
            if not action_name in self.actions_args:
                ret = self.add_action(action_name)
                if ret: rospy.loginfo( 'Adding Action %s', action_name)

        for action_name in self.actions_args:
            if not action_name in action_names:
                ret = self.del_action(action_name)
                if ret: rospy.loginfo ( 'Removing Action %s', action_name)

        # Updating the list of actions
        self.actions_args = action_names
