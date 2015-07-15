# -*- coding: utf-8 -*-
from __future__ import absolute_import

from rosinterface import ActionBack
from rostful_node.ros_interface import get_suffix, CONFIG_PATH, SRV_PATH, MSG_PATH, ACTION_PATH

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
from flask_restful import reqparse
import flask_restful as restful
import flask_login as login

from webargs.flaskparser import FlaskParser, use_kwargs
from webargs import Arg

parser = FlaskParser()

import urllib

"""
View for frontend pages
"""
# TODO: maybe consider http://www.flaskapi.org/

class FrontEnd(MethodView):
    def __init__(self, ros_node):
        super(FrontEnd, self).__init__()
        self.ros_if = ros_node.ros_if  # getting ros interface
        self.rocon_if = ros_node.rocon_if  # getting rocon interface

    #TMP @login.login_required
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
TMP : these should ideally be provided by a Ros node ( rostful-node )
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

#TODO : use rostfulnode instead of direct libraries
#TODO : get worker name and send through celery to support multiple workers
class BackEnd(restful.Resource):
    def __init__(self, ros_node):
        super(BackEnd, self).__init__()
        self.ros_if = ros_node.ros_if  #getting only ros_if for now in backend (TMP).

    # TODO: think about login rest service before disabling REST services if not logged in
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
                return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
            else:
                return make_response(dfile.tostring(suppress_formats=True), 200)  #, content_type='text/plain')

        if not suffix:
            if not self.ros_if.topics.has_key(path):
                if self.ros_if.services.has_key(path):
                    service = self.ros_if.services[path]

                    msg = service.call()
                else:
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

import datetime
import aniso8601  # using this since it s already in our dependencies
class Scheduler(restful.Resource):
    def __init__(self, logger, celery):
        super(Scheduler, self).__init__()
        self.logger = logger
        self.celery = celery
        self.celery.loader.import_default_modules()  # needed to get the CELERY_IMPORTS task in the registry


    def get(self, rurl):
        self.logger.info('GET on backend with rurl = %r', rurl)
        try:
            rpath = rurl.split('/')
            if rpath[0] == 'status':
                #TODO : get task dynamically from celery set

                task = self.celery.tasks['gopher_rocon.celery_tasks.delivery_rapp'].AsyncResult(rpath[1])  # rpath[1] should be the task_id
                self.logger.info('TASK content = %r', task)
                if task.state == 'SUCCESS':
                    # job did not start yet
                    response = {
                        'state': task.state,
                        'result': task.result,
                    }
                else:
                    # something went wrong in the background job
                    response = {
                        'state': task.state,
                        'info': task.info,  # this is the exception raised if FAILURE
                    }
                return jsonify(response)

            else:
                restful.abort(404, message="api {} doesnt exist".format(rpath[0]))

            pass
        except Exception, e:
            rospy.logerr('An exception occurred! %s', e)
            return make_response(e, 500)

    def post(self, rurl):
        try:
            rpath = rurl.split('/')

            if rpath[0] == 'delivery':
                # TMP assume schedule/now

                user_args = {
                    'delivery_path': Arg({
                        'place': Arg(str, required=True),
                    }, multiple=True),
                    'eta': Arg(str, default='')  # expecting a ISO-8601 string
                }

                delivery_data = parser.parse(user_args, request, locations=('json', 'form'))

                #useful to debug parameters send with the request
                #json = request.get_json()
                #return jsonify({key: json[key] for key in json})

                self.logger.info('data : %r', delivery_data)

                if 'gopher_rocon.celery_tasks.delivery_rapp' in self.celery.tasks.keys():
                    dt = aniso8601.parse_datetime(delivery_data['eta'])

                    task = self.celery.tasks['gopher_rocon.celery_tasks.delivery_rapp'].apply_async(kwargs=delivery_data, eta=dt)
                    # using status_url to workaround CORS header Location not being accessible from js
                    #TODO : fix this
                    return {'broker_url': task.backend.url, 'status_url': 'http://localhost:8080/api/schedule/status/' + task.id}, 202, {'Location': 'http://localhost:8080/api/schedule/status/' + task.id}  # url_for ?
                else:
                    return {}, 404

        except Exception, e:
            self.logger.info('Exception on schedule with rurl = %r', rurl)
            return make_response(e, 500)

