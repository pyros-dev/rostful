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
from werkzeug.wrappers import Request, Response
from werkzeug.routing import Map, Rule
from werkzeug.exceptions import HTTPException, NotFound
from werkzeug.utils import redirect
from jinja2 import Environment, FileSystemLoader

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
ServiceFront is the class handling the template generation
for the frontend code, so that a web user can easily generate a request
"""
class ServiceFront:
    def __init__(self, service_name, service_type):
        raise NotImplemented #TODO

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

        self.msgtype = definitions.get_msg_dict(self.rostype)
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
TopicFront is the class handling the template generation
for the frontend code, so that a web user can easily generate a request
"""
class TopicFront:
    def __init__(self, topic_name, topic_type, allow_pub=True, allow_sub=True, queue_size=1):
        raise NotImplemented #TODO


"""
ActionBack is the class handling conversion from REST API to ROS Topic

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

        self.status_msg = deque([], queue_size)
        self.status_sub = rospy.Subscriber(self.name + '/' +self.STATUS_SUFFIX, actionlib_msgs.msg.GoalStatusArray, self.status_callback)

        self.result_msg = deque([], queue_size)
        self.result_sub = rospy.Subscriber(self.name + '/' + self.RESULT_SUFFIX, self.rostype_action_result, self.result_callback)

        self.feedback_msg = deque([], queue_size)
        self.feedback_sub = rospy.Subscriber(self.name + '/' +self.FEEDBACK_SUFFIX, self.rostype_action_feedback, self.feedback_callback)

        self.goal_pub = rospy.Publisher(self.name + '/' + self.GOAL_SUFFIX, self.rostype_action_goal)
        self.cancel_pub = rospy.Publisher(self.name + '/' +self.CANCEL_SUFFIX, actionlib_msgs.msg.GoalID)

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
        self.goal_pub.publish(msg)
        return

    def publish_cancel(self, msg):
        self.cancel_pub.publish(msg)
        return

    def publish(self, suffix, msg):
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

"""
ActionFront is the class handling the template generation
for the frontend code, so that a web user can easily generate a request
"""
class ActionFront:
    def __init__(self, action_name, action_type, queue_size=1):
        raise NotImplemented #TODO


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
                if ret: rospy.logwarn ( 'Exposing Topic %s Pub %r Sub %r', topic_name, allow_pub, allow_sub)

        # Removing extra ones
        for topic_name in self.topics_args:
            if not topic_name in topic_names:
                ret = self.del_topic(topic_name)
                if ret: rospy.logwarn ( 'Removing %s', topic_name)

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
        if not action_names:
            return
        for action_name in action_names:
            if not action_name in self.actions_args:
                ret = self.add_action(action_name)
                if ret: rospy.loginfo( 'Adding action %s', action_name)

        for action_name in self.actions_args:
            if not action_name in action_names:
                ret = self.del_action(action_name)
                if ret: rospy.loginfo ( 'Removing %s', action_name)

        # Updating the list of actions
        self.actions_args = action_names


from flask import Flask, request, make_response, render_template
from flask.views import MethodView
from flask_restful import Resource, Api, reqparse

"""
View for frontend pages
"""
class FrontEnd(MethodView):

    def __init__(self, ros_if):
        super(FrontEnd, self).__init__()
        self.ros_if = ros_if

    def get(self, rosname = None):
        rospy.logwarn('in FrontEnd')
        if not rosname :
            return render_template('turtle.html', topics=self.ros_if.topics, services=self.ros_if.services, actions=self.ros_if.actions )
        else :
            if not self.ros_if.topics.has_key(rosname):
                rospy.logwarn('rosname not found : %s', rosname)
                rospy.logwarn('list of topics :')
                for k in self.ros_if.topics :
                    rospy.logwarn('  %s',k)

                for action_suffix in [ActionBack.STATUS_SUFFIX,ActionBack.RESULT_SUFFIX,ActionBack.FEEDBACK_SUFFIX]:
                    action_name = rosname[:-(len(action_suffix)+1)]
                    if rosname.endswith('/' + action_suffix) and self.ros_if.actions.has_key(action_name):
                        action = self.ros_if.actions[action_name]
                        msg = action.get(action_suffix)
                        break
                    else:
                        return make_response('',404)
            else:
                topic = self.ros_if.topics[rosname]

                return render_template('topic.html', topic=topic )


            return render_template('turtle.html', topics=self.ros_if.topics, services=self.ros_if.services, actions=self.ros_if.actions )


"""
View for backend pages
"""
class BackEnd(Resource):

    def __init__(self, ros_if):
        super(BackEnd, self).__init__()
        self.ros_if = ros_if

    def get(self, rosname):

        rospy.logwarn('in BackEnd')

        parser = reqparse.RequestParser()
        parser.add_argument('full', type=bool)
        parser.add_argument('json', type=bool)
        args = parser.parse_args()

        path = rosname
        full = args['full']

        json_suffix = '.json'
        if path.endswith(json_suffix):
            path = path[:-len(json_suffix)]
            jsn = True
        else:
            jsn = args['json']

        suffix = get_suffix(path)

        if path == CONFIG_PATH:
            dfile = definitions.manifest(self.ros_if.services, self.ros_if.topics, self.ros_if.actions, full=full)
            if jsn:
                return make_response( str(dfile.tojson()), 200)#, content_type='application/json')
            else:
                return make_response( dfile.tostring(suppress_formats=True), 200)#, content_type='text/plain')

        if not suffix:
            if not self.ros_if.topics.has_key(path):
                for action_suffix in [ActionBack.STATUS_SUFFIX,ActionBack.RESULT_SUFFIX,ActionBack.FEEDBACK_SUFFIX]:
                    action_name = path[:-(len(action_suffix)+1)]
                    if path.endswith('/' + action_suffix) and self.ros_if.actions.has_key(action_name):
                        action = self.ros_if.actions[action_name]
                        msg = action.get(action_suffix)
                        break
                else:
                    return make_response('',404)
            else:
                topic = self.ros_if.topics[path]

                if not topic.allow_sub:
                    return make_response('',405)

                msg = topic.get()

            rospy.logwarn('mimetypes : %s' , request.accept_mimetypes )

            if request_wants_ros(request):
                content_type = ROS_MSG_MIMETYPE
                output_data = StringIO()
                if msg is not None:
                    msg.serialize(output_data)
                output_data = output_data.getvalue()
            else: # we default to json
                rospy.logwarn('sending back json')
                content_type = 'application/json'
                output_data = msgconv.extract_values(msg) if msg is not None else None
                output_data = json.dumps(output_data)

            return make_response(output_data, 200) #,content_type=content_type)

        path = path[:-(len(suffix)+1)]

        if suffix == MSG_PATH and self.ros_if.topics.has_key(path):
                return make_response(definitions.get_topic_msg(self.ros_if.topics[path]), 200) #, content_type='text/plain')
        elif suffix == SRV_PATH and self.ros_if.services.has_key(path):
                return make_response(definitions.get_service_srv(self.ros_if.services[path]), 200) #content_type='text/plain')
        elif suffix == ACTION_PATH and self.ros_if.actions.has_key(path):
                return make_response(definitions.get_action_action(self.ros_if.actions[path]), 200) #content_type='text/plain')
        elif suffix == CONFIG_PATH:
            if self.ros_if.services.has_key(path):
                service_name = path

                service = self.ros_if.services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200) #, content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200) # content_type='text/plain')
            elif self.ros_if.topics.has_key(path):
                topic_name = path

                topic = self.ros_if.topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200) #content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200) #content_type='text/plain')
            elif self.ros_if.actions.has_key(path):
                action_name = path

                action = self.ros_if.actions[action_name]
                dfile = definitions.describe_action(action_name, action, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)#, content_type='application/json')
                else:
                    return make_response( dfile.tostring(suppress_formats=True), 200) #, content_type='text/plain')
            else:
                for suffix in [Action.STATUS_SUFFIX,Action.RESULT_SUFFIX,Action.FEEDBACK_SUFFIX,Action.GOAL_SUFFIX,Action.CANCEL_SUFFIX]:
                    if path.endswith('/' + suffix):
                        path = path[:-(len(suffix)+1)]
                        if self.ros_if.actions.has_key(path):
                            action_name = path

                            action = self.ros_if.actions[action_name]
                            dfile = definitions.describe_action_topic(action_name, suffix, action, full=full)

                            if jsn:
                                return make_response(str(dfile.tojson()), 200) #content_type='application/json')
                            else:
                                return make_response(dfile.tostring(suppress_formats=True), 200) #content_type='text/plain')
                return make_response('',404)
        else:
            return make_response('',404)

    def post(self, rosname):

        try:
            length = int(request.environ['CONTENT_LENGTH'])
            content_type = request.environ['CONTENT_TYPE'].split(';')[0].strip()
            use_ros = content_type == ROS_MSG_MIMETYPE

            if self.ros_if.services.has_key(rosname):
                mode = 'service'
                service = self.ros_if.services[rosname]
                input_msg_type = service.rostype_req
            elif self.ros_if.topics.has_key(rosname):
                mode = 'topic'
                topic = self.ros_if.topics[rosname]
                if not topic.allow_pub:
                    return make_response('',405)
                input_msg_type = topic.rostype
            else:
                for suffix in [Action.GOAL_SUFFIX,Action.CANCEL_SUFFIX]:
                    action_name = rosname[:-(len(suffix)+1)]
                    if rosname.endswith('/' + suffix) and self.ros_if.actions.has_key(action_name):
                        mode = 'action'
                        action_mode = suffix
                        action = self.ros_if.actions[action_name]
                        input_msg_type = action.get_msg_type(suffix)
                        break
                else:
                    return make_response('',404)

            input_data = request.environ['wsgi.input'].read(length)

            input_msg = input_msg_type()
            if use_ros:
                input_msg.deserialize(input_data)
            else:
                input_data = json.loads(input_data)
                input_data.pop('_format', None)
                msgconv.populate_instance(input_data, input_msg)

            ret_msg = None
            if mode == 'service':
                rospy.logwarn('calling service %s with msg : %s', service.name, input_msg)
                ret_msg = service.call(input_msg)
            elif mode == 'topic':
                rospy.logwarn('publishing \n%s to topic %s', input_msg, topic.name)
                topic.publish(input_msg)
                return make_response('{}', 200)# content_type='application/json')
            elif mode == 'action':
                rospy.logwarn('publishing %s to action %s', input_msg, action.name)
                action.publish(action_mode, input_msg)
                return make_response('{}', 200)# content_type='application/json')

            if use_ros:
                content_type = ROS_MSG_MIMETYPE
                output_data = StringIO()
                ret_msg.serialize(output_data)
                output_data = output_data.getvalue()
            else:
                output_data = msgconv.extract_values(ret_msg)
                output_data['_format'] = 'ros'
                output_data = json.dumps(output_data)
                content_type = 'application/json'

            return make_response( output_data, 200)#, content_type=content_type)
        except Exception, e:
            rospy.logerr( 'An exception occurred! %s', e )
            return make_response( e, 500)




#~ @app.route('/login', methods=['GET', 'POST'])
#~ def login():
    #~ error = None
    #~ if request.method == 'POST':
        #~ if request.form['username'] != app.config['USERNAME']:
            #~ error = 'Invalid username'
        #~ elif request.form['password'] != app.config['PASSWORD']:
            #~ error = 'Invalid password'
        #~ else:
            #~ session['logged_in'] = True
            #~ flash('You were logged in')
            #~ return redirect(url_for('show_entries'))
    #~ return render_template('login.html', error=error)
#~
#~
#~ @app.route('/logout')
#~ def logout():
    #~ session.pop('logged_in', None)
    #~ flash('You were logged out')
    #~ return redirect(url_for('hello_world'))


import argparse

def servermain(with_static=True):
    rospy.init_node('rostful_server', anonymous=True, disable_signals=True)

    parser = argparse.ArgumentParser()

    #parser.add_argument('--services', '--srv', nargs='+', help='Services to advertise')
    #parser.add_argument('--topics', nargs='+', help='Topics to both publish and subscribe')
    #parser.add_argument('--publishes', '--pub', nargs='+', help='Topics to publish via web services')
    #parser.add_argument('--subscribes', '--sub', nargs='+', help='Topics to allowing publishing to via web services')
    #parser.add_argument('--actions', nargs='+', help='Actions to advertise')

    parser.add_argument('--host', default='')
    parser.add_argument('-p', '--port', type=int, default=8080)

    args = parser.parse_args(rospy.myargv()[1:])

    try:

        ros_if = ROSIF()
        #ros_if.add_services(args.services)
        #ros_if.add_topics(args.topics)
        #ros_if.add_topics(args.publishes, allow_pub=False)
        #ros_if.add_topics(args.subscribes, allow_sub=False)
        #ros_if.add_actions(args.actions)

        rostfront = FrontEnd.as_view('frontend', ros_if)
        rostback = BackEnd.as_view('backend', ros_if)

        #because apparently ROS start python node from ~user/.ros, and it obviously cant find templates there
        app = Flask('rostful',
            static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
            template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates')
        )

        # Load default config and override config from an environment variable
        app.config.update(dict(
            DEBUG=True,
            USERNAME='admin',
            PASSWORD='p@ssw0rd'
        ))
        app.config.from_envvar('ROSTFUL_SETTINGS', silent=True)

        app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])
        app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
        app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET','POST'])
        api = Api(app)

        rospy.loginfo('Starting server on port %d', args.port)
        app.run(port=8080,debug=True)

    except KeyboardInterrupt:
        rospy.loginfo('Shutting down the server')
        rospy.signal_shutdown('Closing')
