# -*- coding: utf-8 -*-
from __future__ import absolute_import
from concurrent import futures
import threading

from .ros_interface import ActionBack, get_suffix, CONFIG_PATH, SRV_PATH, MSG_PATH, ACTION_PATH
from .ros_node import RosNode

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible
import rospy
import json

from StringIO import StringIO

from rosinterface import message_conversion as msgconv
from rosinterface import definitions

from rosinterface.util import ROS_MSG_MIMETYPE, request_wants_ros, get_query_bool

import os
import urlparse
####

from flask import Flask, request, make_response, render_template, jsonify, redirect
from flask.views import MethodView

import flask_security as security
import flask_cors as cors
import flask_restful as restful
import flask_login as login


import urllib

"""
View for frontend pages
"""


class FrontEnd(MethodView):
    def __init__(self, ros_node):
        super(FrontEnd, self).__init__()
        self.ros_if = ros_node.ros_if  # getting ros interface
        self.rocon_if = ros_node.rocon_if  # getting rocon interface

    @login.login_required
    def get(self, rosname=None):
        rospy.logwarn('in FrontEnd with rosname: %r', rosname)
        if not rosname:
            rospy.logwarn('%r', self.ros_if.topics)
            if self.rocon_if:
                return render_template('index.html',
                                       pathname2url=urllib.pathname2url,
                                       topics=self.ros_if.topics,
                                       services=self.ros_if.services,
                                       actions=self.ros_if.actions,
                                       rapp_namespaces=self.rocon_if.rapps_namespaces,
                                       interactions=self.rocon_if.interactions)
            else:
                return render_template('index.html',
                                       pathname2url=urllib.pathname2url,
                                       topics=self.ros_if.topics,
                                       services=self.ros_if.services,
                                       actions=self.ros_if.actions)

        else:
            if self.rocon_if and self.rocon_if.interactions.has_key(rosname):
                mode = 'interaction'
                interaction = self.rocon_if.interactions[rosname]
                result = self.rocon_if.request_interaction(rosname)
                if result.result:
                    if interaction.name.startswith('web_app'):
                        iname = interaction.name[7:].strip("()")
                        rospy.logwarn("Redirecting to WebApp at %r", iname)
                        return render_template('interaction.html', interaction=interaction)
                        #return redirect(iname, code=302)
                    else:
                        return render_template('interaction.html', interaction=interaction)
                else:
                    return jsonify(result), 401

            elif self.rocon_if and self.rocon_if.rapps_namespaces.has_key(rosname):
                mode = 'rapp_namespace'
                rapp_ns = self.rocon_if.rapps_namespaces[rosname]
                return render_template('rapp_namespace.html', rapp_ns=rapp_ns)
            elif self.ros_if.services.has_key(rosname):
                mode = 'service'
                service = self.ros_if.services[rosname]
                return render_template('service.html', service=service)
            elif self.ros_if.topics.has_key(rosname):
                mode = 'topic'
                topic = self.ros_if.topics[rosname]
                return render_template('topic.html', topic=topic)
            elif self.ros_if.actions.has_key(rosname):
                mode = 'action'
                action = self.ros_if.actions[rosname]
                return render_template('action.html', action=action)
            else:
                return '', 404


"""
Additional REST services provided by Rostful itself
"""
class Rostful(restful.Resource):
    def __init__(self, ros_node):
        super(Rostful, self).__init__()
        self.ros_if = ros_node.ros_if  # getting ros interface
        self.rocon_if = ros_node.rocon_if  # getting rocon interface

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rostful_name=None):
        rospy.logwarn('in Rostful with rostful_name: %r', rostful_name)
        if not rostful_name:
            return make_response(jsonify(name="Rostful",
                                         description="REST Services for ROS",
                                         version="v0.1"))
        else:
            spliturl = rostful_name.split('/')
            if len(spliturl) > 0 and spliturl[0] == 'interactions' and self.rocon_if:
                if len(spliturl) > 1 and spliturl[1] in self.rocon_if.interactions:
                    return make_response(jsonify(self.rocon_if.interactions[spliturl[1]]))
                else:
                    return make_response(jsonify(self.rocon_if.interactions))

            if len(spliturl) > 0 and spliturl[0] == 'rapp_namespaces' and self.rocon_if:
                if len(spliturl) > 1 and spliturl[1] in self.rocon_if.rapps_namespaces:
                    return make_response(jsonify(self.rocon_if.rapps_namespaces[spliturl[1]]))
                else:
                    return make_response(jsonify(self.rocon_if.rapps_namespaces))

            if len(spliturl) > 0 and spliturl[0] == 'actions':
                if len(spliturl) > 1 and spliturl[1] in self.ros_if.actions:
                    return make_response(jsonify(self.ros_if.actions[spliturl[1]]))
                else:
                    return make_response(jsonify(self.ros_if.actions))

            if len(spliturl) > 0 and spliturl[0] == 'services':
                if len(spliturl) > 1 and spliturl[1] in self.ros_if.services:
                    return make_response(jsonify(self.ros_if.services[spliturl[1]]))
                else:
                    return make_response(jsonify(self.ros_if.services))

            if len(spliturl) > 0 and spliturl[0] == 'topics':
                if len(spliturl) > 1 and spliturl[1] in self.ros_if.topics:
                    return make_response(jsonify(self.ros_if.topics[spliturl[1]]))
                else:
                    return make_response(jsonify(self.ros_if.topics))

            else:
                return make_response('', 404)



"""
View for backend pages
"""


class BackEnd(restful.Resource):
    def __init__(self, ros_node):
        super(BackEnd, self).__init__()
        self.ros_if = ros_node.ros_if  #getting only ros_if for now in backend (TMP).

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rosname):

        rospy.logwarn('in BackEnd with rosname: %r', rosname)

        parser = restful.reqparse.RequestParser()
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
                return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
            else:
                return make_response(dfile.tostring(suppress_formats=True), 200)  #, content_type='text/plain')

        if not suffix:
            if not self.ros_if.topics.has_key(path):
                for action_suffix in [ActionBack.STATUS_SUFFIX, ActionBack.RESULT_SUFFIX, ActionBack.FEEDBACK_SUFFIX]:
                    action_name = path[:-(len(action_suffix) + 1)]
                    if path.endswith('/' + action_suffix) and self.ros_if.actions.has_key(action_name):
                        action = self.ros_if.actions[action_name]
                        msg = action.get(action_suffix)
                        break
                else:
                    return make_response('', 404)
            else:
                topic = self.ros_if.topics[path]

                if not topic.allow_sub:
                    return make_response('', 405)

                msg = topic.get()

            rospy.logwarn('mimetypes : %s', request.accept_mimetypes)

            if request_wants_ros(request):
                content_type = ROS_MSG_MIMETYPE
                output_data = StringIO()
                if msg is not None:
                    msg.serialize(output_data)
                output_data = output_data.getvalue()
            else:  # we default to json
                rospy.logwarn('sending back json')
                content_type = 'application/json'
                output_data = msgconv.extract_values(msg) if msg is not None else None
                output_data = json.dumps(output_data)

            return make_response(output_data, 200)  #,content_type=content_type)

        path = path[:-(len(suffix) + 1)]

        if suffix == MSG_PATH and self.ros_if.topics.has_key(path):
            return make_response(definitions.get_topic_msg(self.ros_if.topics[path]),
                                 200)  #, content_type='text/plain')
        elif suffix == SRV_PATH and self.ros_if.services.has_key(path):
            return make_response(definitions.get_service_srv(self.ros_if.services[path]),
                                 200)  #content_type='text/plain')
        elif suffix == ACTION_PATH and self.ros_if.actions.has_key(path):
            return make_response(definitions.get_action_action(self.ros_if.actions[path]),
                                 200)  #content_type='text/plain')
        elif suffix == CONFIG_PATH:
            if self.ros_if.services.has_key(path):
                service_name = path

                service = self.ros_if.services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200)  # content_type='text/plain')
            elif self.ros_if.topics.has_key(path):
                topic_name = path

                topic = self.ros_if.topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)  #content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200)  #content_type='text/plain')
            elif self.ros_if.actions.has_key(path):
                action_name = path

                action = self.ros_if.actions[action_name]
                dfile = definitions.describe_action(action_name, action, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200)  #, content_type='text/plain')
            else:
                for suffix in [ActionBack.STATUS_SUFFIX, ActionBack.RESULT_SUFFIX, ActionBack.FEEDBACK_SUFFIX,
                               ActionBack.GOAL_SUFFIX, ActionBack.CANCEL_SUFFIX]:
                    if path.endswith('/' + suffix):
                        path = path[:-(len(suffix) + 1)]
                        if self.ros_if.actions.has_key(path):
                            action_name = path

                            action = self.ros_if.actions[action_name]
                            dfile = definitions.describe_action_topic(action_name, suffix, action, full=full)

                            if jsn:
                                return make_response(str(dfile.tojson()), 200)  #content_type='application/json')
                            else:
                                return make_response(dfile.tostring(suppress_formats=True),
                                                     200)  #content_type='text/plain')
                return make_response('', 404)
        else:
            return make_response('', 404)

    # TODO: think about login rest service before disabling REST services if not logged in
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
                    return make_response('', 405)
                input_msg_type = topic.rostype
            else:
                rospy.logwarn('ACTION')
                for suffix in [ActionBack.GOAL_SUFFIX, ActionBack.CANCEL_SUFFIX]:
                    action_name = rosname[:-(len(suffix) + 1)]
                    if rosname.endswith('/' + suffix) and self.ros_if.actions.has_key(action_name):
                        mode = 'action'
                        action_mode = suffix
                        rospy.logwarn('MODE:%r', action_mode)
                        action = self.ros_if.actions[action_name]
                        input_msg_type = action.get_msg_type(suffix)
                        rospy.logwarn('input_msg_type:%r', input_msg_type)
                        break
                else:
                    return make_response('', 404)

            input_data = request.environ['wsgi.input'].read(length)

            input_msg = input_msg_type()
            rospy.logwarn('input_msg:%r', input_msg)
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
                return make_response('{}', 200)  # content_type='application/json')
            elif mode == 'action':
                rospy.logwarn('publishing %s to action %s', input_msg, action.name)
                action.publish(action_mode, input_msg)
                return make_response('{}', 200)  # content_type='application/json')

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

            return make_response(output_data, 200)  #, content_type=content_type)
        except Exception, e:
            rospy.logerr('An exception occurred! %s', e)
            return make_response(e, 500)


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
        db.init_app(self.app)
        self.db = db

        # Setup Flask-Security
        self.user_datastore = security.SQLAlchemyUserDatastore(self.db, db_models.User, db_models.Role)
        self.security = security.Security(self.app, self.user_datastore)

        # One of the simplest configurations. Exposes all resources matching /* to
        # CORS and allows the Content-Type header, which is necessary to POST JSON
        # cross origin.
        self.cors = cors.CORS(self.app, resources=r'/*', allow_headers='Content-Type')

    def launch(self, ros_args):
        self.ros_node = RosNode(ros_args)
        rostfront = FrontEnd.as_view('frontend', self.ros_node)
        rostback = BackEnd.as_view('backend', self.ros_node)
        rostful = Rostful.as_view('rostful', self.ros_node)

        # TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
        self.app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])
        self.app.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
        self.app.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])
        self.api = restful.Api(self.app)

    def shutdown(self):
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()


