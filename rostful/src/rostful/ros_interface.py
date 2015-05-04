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

from rosinterface import message_conversion as msgconv
from rosinterface import definitions, util
from rosinterface.util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse

from rosinterface.action import ActionBack
from rosinterface.service import ServiceBack
from rosinterface.topic import TopicBack

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
class RosInterface:
    #Protecting ROSIF to make sure it s always cleaned up properly
    class ROSIF():
        def __init__(self, ros_args):
            #we initialize the node here, passing ros parameters
            rospy.init_node('rostful_server', argv=ros_args, anonymous=True, disable_signals=True)

            #current services topics and actions exposed
            self.services = {}
            self.topics = {}
            self.actions = {}
            #last requested services topics and actions to be exposed
            self.services_args = {}
            self.topics_args = {}
            self.actions_args = {}

            full_param_name = rospy.search_param('topics')
            param_value = rospy.get_param(full_param_name)
            rospy.logwarn('Parameter %s is %s', rospy.resolve_name('topics'), full_param_name)

            topics_args = ast.literal_eval(rospy.get_param('~topics', "[]"))
            rospy.logwarn('Parameter %s has value %s', rospy.resolve_name('~topics'), rospy.get_param('~topics', "[]"))
            services_args = ast.literal_eval(rospy.get_param('~services', "[]"))
            rospy.logwarn('Parameter %s has value %s', rospy.resolve_name('~services'), rospy.get_param('~services', "[]"))
            actions_args = ast.literal_eval(rospy.get_param('~actions', "[]"))
            rospy.logwarn('Parameter %s has value %s', rospy.resolve_name('~actions'), rospy.get_param('~actions', "[]"))

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

        """
        Does all necessary cleanup to bring down RosInterface
        """
        def cleanup(self):
            #TODO
            pass

    #Managing the context
    def __init__(self, ros_args):
        self.ros_args = ros_args

    def __enter__(self):
        #returning the nested class instance
        self.interface = self.ROSIF(self.ros_args)
        return self.interface

    def __exit__(self, type, value, traceback):
        self.interface.cleanup()
        rospy.signal_shutdown('Closing')
