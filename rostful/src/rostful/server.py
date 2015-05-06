from __future__ import absolute_import
from .ros_interface import RosInterface, ActionBack, get_suffix, CONFIG_PATH, SRV_PATH, MSG_PATH, ACTION_PATH

#TODO : remove ROS usage here, keep this a pure Flask App as much as possible
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
from rosinterface import definitions

from rosinterface.util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse
####

from flask import Flask, request, make_response, render_template
from flask.views import MethodView
from flask.ext.sqlalchemy import SQLAlchemy
from flask.ext.security import Security, SQLAlchemyUserDatastore
from flask_restful import Resource, Api, reqparse
from flask.ext.login import LoginManager, login_required

from rosinterface.service import ServiceBack
from .interaction import Interaction
from .interaction_table import InteractionsTable

import rocon_interactions
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri

"""
View for frontend pages
"""
class FrontEnd(MethodView):

    def _set_remocon_services(self, interactions_namespace):
        """
            setting up remocon-interaction manager apis. and check if the services are available

            :param str interactions_namespace : namespace to contact interaction manager

            :returns: remocon service apis
            :rtype: dict
        """
        remocon_services = {}
        remocon_services['get_interactions'] = interactions_namespace + '/' + 'get_interactions'
        remocon_services['get_roles'] = interactions_namespace + '/' + 'get_roles'
        remocon_services['request_interaction'] = interactions_namespace + '/' + 'request_interaction'

        for service_name in remocon_services.keys():
            if not rocon_python_comms.service_is_available(remocon_services[service_name]):
                raise rocon_python_comms.NotFoundException("'%s' service is not validated" % service_name)

        return remocon_services


    def __init__(self, ros_if):
        super(FrontEnd, self).__init__()
        self.ros_if = ros_if
        self.rocon_uri = rocon_uri.parse(
            "rocon:/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "|" + rocon_std_msgs.Strings.OS_CHROME
        )
        self.platform_info = rocon_std_msgs.PlatformInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                         uri=str(self.rocon_uri),
                                                         icon=rocon_std_msgs.Icon()
                                                         )

        self.interactions_table = InteractionsTable()
        try:
            rospy.logdebug("Interactive Client : Get interactions service Handle")
            interactions_namespace = rocon_python_comms.find_service_namespace('get_interactions', 'rocon_interaction_msgs/GetInteractions', unique=True)
            remocon_services = self._set_remocon_services(interactions_namespace)
        except rocon_python_comms.MultipleFoundException as e:
            message = "multiple interactions' publications and services [%s] are found. Please check services" % str(e)
            rospy.logerror("InteractiveClientInterface : %s" % message)
            return (False, message)
        except rocon_python_comms.NotFoundException as e:
            message = "failed to find all of the interactions' publications and services for [%s]" % str(e)
            rospy.logerror("InteractiveClientInterface : %s" % message)
            return (False, message)

        self.get_interactions_service_proxy = rospy.ServiceProxy(remocon_services['get_interactions'], rocon_interaction_srvs.GetInteractions)
        self.get_roles_service_proxy = rospy.ServiceProxy(remocon_services['get_roles'], rocon_interaction_srvs.GetRoles)
        self.request_interaction_service_proxy = rospy.ServiceProxy(remocon_services['request_interaction'], rocon_interaction_srvs.RequestInteraction)

        call_result = self.get_interactions_service_proxy([], self.platform_info.uri)
        for msg in call_result.interactions:
            self.interactions_table.append(Interaction(msg))

        rospy.logwarn ('Loaded %r Interactions', len(self.interactions_table))
        self.interactions= { i.name: i for i in self.interactions_table}


    def get(self, rosname = None):
        rospy.logwarn('in FrontEnd with rosname: %r', rosname)
        if not rosname :
            return render_template('index.html', topics=self.ros_if.topics, services=self.ros_if.services, actions=self.ros_if.actions, interactions=self.interactions )
        else :
            if self.ros_if.services.has_key(rosname):
                mode = 'service'
                service = self.ros_if.services[rosname]
                return render_template('service.html', service=service )
            elif self.ros_if.topics.has_key(rosname):
                mode = 'topic'
                topic = self.ros_if.topics[rosname]
                return render_template('topic.html', topic=topic )
            elif self.ros_if.actions.has_key(rosname):
                mode = 'action'
                action = self.ros_if.actions[rosname]
                return render_template('action.html', action=action )
            else :
                return '', 404


"""
View for backend pages
"""
class BackEnd(Resource):

    def __init__(self, ros_if):
        super(BackEnd, self).__init__()
        self.ros_if = ros_if

    def get(self, rosname):

        rospy.logwarn('in BackEnd with rosname: %r', rosname)

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
                for suffix in [ActionBack.STATUS_SUFFIX,ActionBack.RESULT_SUFFIX,ActionBack.FEEDBACK_SUFFIX,ActionBack.GOAL_SUFFIX,ActionBack.CANCEL_SUFFIX]:
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
            rospy.logwarn('POST')
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
                rospy.logwarn('ACTION')
                for suffix in [ActionBack.GOAL_SUFFIX,ActionBack.CANCEL_SUFFIX]:
                    action_name = rosname[:-(len(suffix)+1)]
                    if rosname.endswith('/' + suffix) and self.ros_if.actions.has_key(action_name):
                        mode = 'action'
                        action_mode = suffix
                        rospy.logwarn('MODE:%r', action_mode)
                        action = self.ros_if.actions[action_name]
                        input_msg_type = action.get_msg_type(suffix)
                        rospy.logwarn('input_msg_type:%r', input_msg_type)
                        break
                else:
                    return make_response('',404)

            input_data = request.environ['wsgi.input'].read(length)

            input_msg = input_msg_type()
            rospy.logwarn('input_msg:%r',input_msg)
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


from . import db_models
from .db_models import db


class Server():
    #TODO : pass config file from command line here
    def __init__(self):

        #TODO : change into application factory (https://github.com/miguelgrinberg/Flask-Migrate/issues/45)
        #because apparently ROS start python node from ~user/.ros, and it obviously cant find templates there
        self.app = Flask('rostful',
            static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
            template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
            instance_relative_config=True
        )

        self.app.config.from_object('config.default')
        self.app.config.from_object('config.development')
        #TODO : flexible config by chosing file
        #TODO : flexible config by getting file from instance folder
        #TODO : flexible config by getting env var

        #initializes DB
        self.db = db.init_app(self.app)

        # Setup Flask-Security
        user_datastore = SQLAlchemyUserDatastore(self.db, db_models.User, db_models.Role)
        security = Security(self.app, user_datastore)

    def launch(self,ros_args):
        self.ros_if = RosInterface(ros_args)
        rostfront = FrontEnd.as_view('frontend', self.ros_if)
        rostback = BackEnd.as_view('backend', self.ros_if)

        self.app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET','POST'])
        self.api = Api(self.app)


