# -*- coding: utf-8 -*-
from __future__ import absolute_import

from rosinterface import ActionBack
from rostful_node.ros_interface import get_suffix, CONFIG_PATH, SRV_PATH, MSG_PATH, ACTION_PATH

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible
import rospy
import json
import logging
import logging.handlers

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
    def __init__(self, ros_node_client, logger, debug):
        super(FrontEnd, self).__init__()
        self.node_client = ros_node_client
        self.logger = logger

    #TMP @login.login_required
    def get(self, rosname=None):
        self.logger.debug('in FrontEnd with rosname: %r', rosname)
        if not rosname:
            return render_template('index.html',
                                   pathname2url=urllib.pathname2url,
                                   topics=self.node_client.listtopics(),
                                   services=self.node_client.listsrvs(),
                                   params=self.node_client.listparams(),
                                   actions=self.node_client.listacts(),
                                   rapp_namespaces=self.node_client.namespaces(),
                                   interactions=self.node_client.interactions())
        else:
            rosname = '/' + rosname
            has_rocon = self.node_client.has_rocon()
            # TODO: this isn't very efficient, but don't know a better way to do it
            interactions = self.node_client.interactions()
            namespaces = self.node_client.namespaces()
            services = self.node_client.listsrvs()
            topics = self.node_client.listtopics()
            actions = self.node_client.listacts()
            
            if has_rocon and rosname in interactions:
                mode = 'interaction'
                interaction = interactions[rosname]
                result = self.node_client.interaction(rosname).interaction
                if result.result:
                    if interaction.name.startswith('web_app'):
                        iname = interaction.name[7:].strip("()")
                        self.logger.debug("Redirecting to WebApp at %r", iname)
                        return render_template('interaction.html', interaction=interaction)
                        #return redirect(iname, code=302)
                    else:
                        return render_template('interaction.html', interaction=interaction)
                else:
                    return jsonify(result), 401
            elif has_rocon and rosname in namespaces:
                mode = 'rapp_namespace'
                rapp_ns = namespaces[rosname]
                return render_template('rapp_namespace.html', rapp_ns=rapp_ns)
            elif rosname in services:
                mode = 'service'
                service = services[rosname]
                return render_template('service.html', service=service)
            elif rosname in topics:
                mode = 'topic'
                topic = topics[rosname]
                return render_template('topic.html', topic=topic)
            elif rosname in actions:
                mode = 'action'
                action = actions[rosname]
                return render_template('action.html', action=action)
            else:
                return '', 404


"""
Additional REST services provided by Rostful itself
TMP : these should ideally be provided by a Ros node ( rostful-node ? RosAPI ? )
"""
class Rostful(restful.Resource):
    def __init__(self, ros_node_client, logger, debug):
        super(Rostful, self).__init__()
        self.node_client = ros_node_client
        self.logger = logger

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rostful_name=None):
        self.logger.debug('in Rostful with rostful_name: %r', rostful_name)
        if not rostful_name:
            return make_response(jsonify(name="Rostful",
                                         description="REST Services for ROS",
                                         version="v0.1"))
        else:
            spliturl = rostful_name.split('/')
            has_rocon = self.node_client.has_rocon()
            if len(spliturl) > 0 and spliturl[0] == 'interactions' and has_rocon:
                interactions = self.node_client.interactions()
                if len(spliturl) > 1 and spliturl[1] in interactions:
                    return make_response(jsonify(self.node_client.interaction(spliturl[1]).interaction))
                else:
                    return make_response(jsonify(interactions))

            if len(spliturl) > 0 and spliturl[0] == 'rapp_namespaces' and has_rocon:
                namespaces = self.node_client.namespaces()
                if len(spliturl) > 1 and spliturl[1] in self.rocon_if.rapps_namespaces:
                    return make_response(jsonify(namespaces[spliturl[1]]))
                else:
                    return make_response(jsonify(namespaces))

            if len(spliturl) > 0 and spliturl[0] == 'actions':
                actions = self.node_client.listacts()
                if len(spliturl) > 1 and spliturl[1] in actions:
                    return make_response(jsonify(actions[spliturl[1]]))
                else:
                    return make_response(jsonify(actions))

            if len(spliturl) > 0 and spliturl[0] == 'services':
                services = self.node_client.listsrvs()
                if len(spliturl) > 1 and spliturl[1] in services:
                    return make_response(jsonify(self.node_client.service_call(spliturl[1])))
                else:
                    return make_response(jsonify(services))

            if len(spliturl) > 0 and spliturl[0] == 'topics':
                topics = self.node_client.listtopics()
                if len(spliturl) > 1 and spliturl[1] in topics:
                    return make_response(jsonify(self.node_client.topic_extract(spliturl[1])))
                else:
                    return make_response(jsonify(topics))

            else:
                return make_response('', 404)



"""
View for backend pages
"""

class BackEnd(restful.Resource):   # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/
    def __init__(self, ros_node_client, logger, debug):
        super(BackEnd, self).__init__()
        self.node_client = ros_node_client
        self.logger = logger

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rosname):
        #self.logger.debug('in BackEnd with rosname: %r', rosname)

        # TODO : replace this with webargs ( less buggy )
        parser = reqparse.RequestParser()
        parser.add_argument('full', type=bool)
        parser.add_argument('json', type=bool)
        args = parser.parse_args()

        path = '/' + rosname
        full = args['full']

        json_suffix = '.json'
        if path.endswith(json_suffix):
            path = path[:-len(json_suffix)]
            jsn = True
        else:
            jsn = args['json']

        suffix = get_suffix(path)

        services = self.node_client.listsrvs()
        topics = self.node_client.listtopics()
        actions = self.node_client.listacts()
        params = self.node_client.listparams()
        
        if path == CONFIG_PATH:
            dfile = definitions.manifest(services, topics, actions, full=full)
            if jsn:
                return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
            else:
                return make_response(dfile.tostring(suppress_formats=True), 200)  #, content_type='text/plain')

        if not suffix:
            if path in params:
                msg = self.node_client.param_get(path)
            elif path in services:
                msg = self.node_client.service_call(path)
            elif path in topics:
                if not topics[path].allow_sub:
                    self.logger.warn('405 : %s', path)
                    return make_response('', 405)

                msg = self.node_client.topic_extract(path)
            else:
                for action_suffix in [ActionBack.STATUS_SUFFIX, ActionBack.RESULT_SUFFIX, ActionBack.FEEDBACK_SUFFIX]:
                    action_name = path[:-(len(action_suffix) + 1)]
                    if path.endswith('/' + action_suffix) and action_name in actions:
                        action = actions[action_name]
                        msg = action.get(action_suffix)
                        break
                    else:
                        self.logger.warn('404 : %s', path)
                        return make_response('', 404)

            #self.logger.debug('mimetypes : %s', request.accept_mimetypes)

            content_type = 'application/json'
            output_data = json.dumps(msg)

            #if request_wants_ros(request):
            #    content_type = ROS_MSG_MIMETYPE
            #    output_data = StringIO()
            #    if msg is not None:
            #        msg.serialize(output_data)
            #    output_data = output_data.getvalue()
            #else:  # we default to json
            #    # self.logger.debug('sending back json')
            #    content_type = 'application/json'
            #    output_data = msgconv.extract_values(msg) if msg is not None else None
            #    output_data = json.dumps(output_data)

            return make_response(output_data, 200)  #,content_type=content_type)

        path = path[:-(len(suffix) + 1)]

        if suffix == MSG_PATH and path in topics:
            return make_response(definitions.get_topic_msg(topics[path]),
                                 200)  #, content_type='text/plain')
        elif suffix == SRV_PATH and self.ros_if.services.has_key(path):
            return make_response(definitions.get_service_srv(services[path]),
                                 200)  #content_type='text/plain')
        elif suffix == ACTION_PATH and self.ros_if.actions.has_key(path):
            return make_response(definitions.get_action_action(actions[path]),
                                 200)  #content_type='text/plain')
        elif suffix == CONFIG_PATH:
            if path in services:
                service_name = path

                service = services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)  #, content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200)  # content_type='text/plain')
            elif path in topics:
                topic_name = path

                topic = topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    return make_response(str(dfile.tojson()), 200)  #content_type='application/json')
                else:
                    return make_response(dfile.tostring(suppress_formats=True), 200)  #content_type='text/plain')
            elif self.ros_if.actions.has_key(path):
                action_name = path

                action = actions[action_name]
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
                        if path in actions:
                            action_name = path

                            action = actions[action_name]
                            dfile = definitions.describe_action_topic(action_name, suffix, action, full=full)

                            if jsn:
                                return make_response(str(dfile.tojson()), 200)  #content_type='application/json')
                            else:
                                return make_response(dfile.tostring(suppress_formats=True),
                                                     200)  #content_type='text/plain')
                return make_response('', 404)
        else:
            self.logger.warn('404 : %s', path)
            return make_response('', 404)

    # TODO: think about login rest service before disabling REST services if not logged in
    def post(self, rosname):

        try:
            rosname = '/' + rosname
            #self.logger.debug('POST')
            length = int(request.environ['CONTENT_LENGTH'])
            content_type = request.environ['CONTENT_TYPE'].split(';')[0].strip()
            use_ros = content_type == ROS_MSG_MIMETYPE

            services = self.node_client.listsrvs()
            topics = self.node_client.listtopics()
            actions = self.node_client.listacts()
            params = self.node_client.listparams()

            if rosname in services:
                mode = 'service'
                service = services[rosname]
                input_msg_type = service.rostype_req
            elif rosname in topics:
                mode = 'topic'
                topic = topics[rosname]
                if not topic.allow_pub:
                    self.logger.warn('405 : %s', rosname)
                    return make_response('', 405)
                input_msg_type = topic.rostype
            elif rosname in params:
                mode = 'param'
                param = params[rosname]
            else:
                #self.logger.debug('ACTION')
                for suffix in [ActionBack.GOAL_SUFFIX, ActionBack.CANCEL_SUFFIX]:
                    action_name = rosname[:-(len(suffix) + 1)]
                    if rosname.endswith('/' + suffix) and action_name in actions:
                        mode = 'action'
                        action_mode = suffix
                        #self.logger.debug('MODE:%r', action_mode)
                        action = actions[action_name]
                        input_msg_type = action.get_msg_type(suffix)
                        #self.logger.debug('input_msg_type:%r', input_msg_type)
                        break
                else:
                    self.logger.warn('404 : %s', rosname)
                    return make_response('', 404)

            # we are now sending via the client node, which will convert the
            # received dict into the correct message type for the service (or
            # break, if it's wrong.)
            input_data = request.environ['wsgi.input'].read(length)
            input_data = json.loads(input_data)
            input_data.pop('_format', None)

            # input_msg = input_msg_type()
            # self.logger.debug('input_msg:%r', input_msg)
            # if use_ros:
            #     input_msg.deserialize(input_data)
            # else:
            #     input_data = json.loads(input_data)
            #     input_data.pop('_format', None)
            #     msgconv.populate_instance(input_data, input_msg)

            ret_msg = None
            if mode == 'service':
                self.logger.debug('calling service %s with msg : %s', service.name, input_data)
                ret_msg = self.node_client.service_call(rosname, input_data)
            elif mode == 'topic':
                self.logger.debug('publishing \n%s to topic %s', input_data, topic.name)
                self.node_client.topic_inject(rosname, input_data)
                return make_response('{}', 200)  # content_type='application/json')
            elif mode == 'param':
                self.logger.debug('setting \n%s param %s', input_data, param.name)
                self.node_client.param_set(rosname, input_data)
                return make_response('{}', 200)  # content_type='application/json')
            elif mode == 'action':
                self.logger.debug('publishing %s to action %s', input_data, action.name)
                self.node_client.action(action.name, action_mode, input_data)
                return make_response('{}', 200)  # content_type='application/json')

            if use_ros:
                content_type = ROS_MSG_MIMETYPE
                output_data = StringIO()
                ret_msg.serialize(output_data)
                output_data = output_data.getvalue()
            elif ret_msg:
                output_data = ret_msg # the returned message is already converted from ros format by the client
                output_data['_format'] = 'ros'
                output_data = json.dumps(output_data)
                content_type = 'application/json'
            else:
                output_data = "{}"

            return make_response(output_data, 200)  #, content_type=content_type)
        except Exception, e:
            self.logger.error('An exception occurred! => 500 %s', e)
            return make_response(e, 500)
