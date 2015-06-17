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
import ast

from rosinterface.action import ActionBack
from rosinterface.service import ServiceBack
from rosinterface.topic import TopicBack

from .ros_watcher import ROSWatcher

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

"""
Interface with ROS.
No inheritance to make sure destructor is called properly.
"""
class RosInterface():
    def __init__(self):
        #current services topics and actions exposed
        self.services = {}
        self.topics = {}
        self.actions = {}
        #current services topics and actions we are waiting for
        self.services_waiting = []
        self.topics_waiting = []
        self.actions_waiting = []
        #last requested services topics and actions to be exposed
        self.services_args = []
        self.topics_args = []
        self.actions_args = []

        topics_args = ast.literal_eval(rospy.get_param('~topics', "[]"))
        services_args = ast.literal_eval(rospy.get_param('~services', "[]"))
        actions_args = ast.literal_eval(rospy.get_param('~actions', "[]"))

        self.expose_topics(topics_args)
        self.expose_services(services_args)
        self.expose_actions(actions_args)

        self.ros_watcher = ROSWatcher(self.topics_change_cb, self.services_change_cb, self.actions_change_cb)
        self.ros_watcher.start()


    def reconfigure(self, config, level):
        rospy.logwarn("""ROSInterface Reconfigure Request: \ntopics : {topics} \nservices : {services} \nactions : {actions}""".format(**config))
        new_topics = ast.literal_eval(config["topics"])
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
                rospy.logwarn('Cannot Expose unknown service %s' % service_name)
                self.services_waiting.append(service_name)
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

        if not self.services.pop(ws_name, None):
            self.services_waiting.remove(service_name)
        return True

    """
    This exposes a list of services as REST API. services not listed here will be removed from the API
    """
    def expose_services(self, service_names):
        #rospy.loginfo('Exposing services : %r', service_names)
        if not service_names:
            return
        for service_name in service_names:
            if not service_name in self.services_args:
                ret = self.add_service(service_name)
                #if ret: rospy.loginfo( 'Exposed Service %s', service_name )

        for service_name in self.services_args:
            if not service_name in service_names:
                ret = self.del_service(service_name)
                #if ret: rospy.loginfo ( 'Removed Service %s', service_name )

        #Updating the list of services
        self.services_args = service_names

    def add_topic(self, topic_name, ws_name=None, topic_type=None, allow_pub=True, allow_sub=True):
        resolved_topic_name = rospy.resolve_name(topic_name)
        if topic_type is None:
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if not topic_type:
                rospy.logwarn('Cannot Expose unknown topic %s' % topic_name)
                self.topics_waiting.append(topic_name)
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

        if not self.topics.pop(ws_name,None) :
            self.topics_waiting.remove(topic_name)
        return True

    """
    This exposes a list of topics as REST API. topics not listed here will be removed from the API
    """
    def expose_topics(self, topic_names, allow_pub=True, allow_sub=True):
        # rospy.loginfo('Exposing topics : %r', topic_names)
        if not topic_names:
            return
        # Adding missing ones
        for topic_name in topic_names:
            if not topic_name in self.topics_args:
                ret = self.add_topic(topic_name, allow_pub=allow_pub, allow_sub=allow_sub)
                # if ret: rospy.loginfo('Exposed Topic %s Pub %r Sub %r', topic_name, allow_pub, allow_sub)

        # Removing extra ones
        for topic_name in self.topics_args:
            if not topic_name in topic_names:
                ret = self.del_topic(topic_name)
                # if ret: rospy.loginfo('Removed Topic %s', topic_name)

        # Updating the list of topics
        self.topics_args = topic_names

    def add_action(self, action_name, ws_name=None, action_type=None):
        if action_type is None:
            resolved_topic_name = rospy.resolve_name(action_name + '/result')
            topic_type, _, _ = rostopic.get_topic_type(resolved_topic_name)
            if not topic_type:
                rospy.logwarn( 'Cannot Expose unknown action %s', action_name )
                self.actions_waiting.append(action_name)
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

        if not self.actions.pop(ws_name,None) :
            self.actions_waiting.remove(action_name)
        return True

    """
    This exposes a list of actions as REST API. actions not listed here will be removed from the API
    """
    def expose_actions(self, action_names):
        #rospy.loginfo('Exposing actions : %r', action_names)
        if not action_names:
            return
        for action_name in action_names:
            if not action_name in self.actions_args:
                ret = self.add_action(action_name)
                #if ret: rospy.loginfo( 'Exposed Action %s', action_name)

        for action_name in self.actions_args:
            if not action_name in action_names:
                ret = self.del_action(action_name)
                #if ret: rospy.loginfo ( 'Removed Action %s', action_name)

        # Updating the list of actions
        self.actions_args = action_names

    def topics_change_cb(self, new_topics, lost_topics):
        # rospy.logwarn('new topics : %r, lost topics : %r', new_topics, lost_topics)
        topics_lst = [t for t in new_topics if t in self.topics_waiting]
        if len(topics_lst) > 0:
            # rospy.logwarn('exposing new topics : %r', topics_lst)
            # Adding missing ones
            for topic_name in topics_lst:
                ret = self.add_topic(topic_name)

        topics_lst = [t for t in lost_topics if t in self.topics_args]
        if len(topics_lst) > 0:
            # rospy.logwarn('hiding lost topics : %r', topics_lst)
            # Removing extra ones
            for topic_name in topics_lst:
                ret = self.del_topic(topic_name)

    def services_change_cb(self, new_services, lost_services):
        # rospy.logwarn('new services : %r, lost services : %r', new_services, lost_services)
        svc_list = [s for s in new_services if s in self.services_waiting]
        if len(svc_list) > 0:
            # rospy.logwarn('exposing new services : %r', svc_list)
            for svc_name in svc_list:
                self.add_service(svc_name)

        svc_list = [s for s in lost_services if s in self.services_args]
        if len(svc_list) > 0:
            # rospy.logwarn('hiding lost services : %r', svc_list)
            for svc_name in svc_list:
                self.del_service(svc_name)

    def actions_change_cb(self, new_actions, lost_actions):
        # rospy.logwarn('new actions : %r, lost actions : %r', new_actions, lost_actions)
        act_list = [a for a in new_actions if a in self.actions_waiting]
        if len(act_list) > 0:
            # rospy.logwarn('exposing new actions : %r', act_list)
            for act_name in act_list:
                self.add_action(act_name)

        act_list = [a for a in lost_actions if a in self.actions_args]
        if len(act_list) > 0:
            # rospy.logwarn('hiding lost actions : %r', act_list)
            for act_name in act_list:
                self.del_action(act_name)
