# -*- coding: utf-8 -*-
from __future__ import absolute_import

import re

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'
ACTION_PATH = '_action'

def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH,SRV_PATH,MSG_PATH,ACTION_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible
import rospy
import json
import logging
import logging.handlers

from StringIO import StringIO

from pyros.rosinterface import definitions


ROS_MSG_MIMETYPE = 'application/vnd.ros.msg'
def ROS_MSG_MIMETYPE_WITH_TYPE(rostype):
    if isinstance(rostype,type):
        name = rostype.__name__
        module = rostype.__module__.split('.')[0]
        rostype = module + '/' + name
    return 'application/vnd.ros.msg; type=%s' % rostype


#req should be a flask request
#TODO : improve package design...
def request_wants_ros(req):
    best = req.accept_mimetypes.best_match([ROS_MSG_MIMETYPE,'application/json'])
    return best == ROS_MSG_MIMETYPE and req.accept_mimetypes[best] > req.accept_mimetypes['application/json']
#implementation ref : http://flask.pocoo.org/snippets/45/


def get_json_bool(b):
    if b:
        return 'true'
    else:
        return 'false'


def get_query_bool(query_string, param_name):
    return re.search(r'(^|&)%s((=(true|1))|&|$)' % param_name,query_string,re.IGNORECASE)


from flask import Flask, request, make_response, render_template, jsonify, redirect
from flask.views import MethodView
from flask_restful import reqparse
import flask_restful as restful
import flask_login as login

from webargs.flaskparser import FlaskParser, use_kwargs

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
                                   topics=self.node_client.topics(),
                                   services=self.node_client.services(),
                                   params=self.node_client.params(),
                                   rapp_namespaces=[],  #self.node_client.namespaces(),
                                   interactions=[],  #self.node_client.interactions()
            )
        else:
            rosname = '/' + rosname
            has_rocon = False #self.node_client.has_rocon()
            # TODO: this isn't very efficient, but don't know a better way to do it
            interactions = [] #self.node_client.interactions()
            namespaces = [] #self.node_client.namespaces()
            services = self.node_client.services()
            topics = self.node_client.topics()
            
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
            else:
                return '', 404



class Rostful(restful.Resource):
    """
    Additional REST services provided by Rostful itself
    TMP : these should ideally be provided by a Ros node ( rostful-node ? RosAPI ? )
    """
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

            if len(spliturl) > 0 and spliturl[0] == 'services':
                services = self.node_client.services()
                if len(spliturl) > 1 and spliturl[1] in services:
                    return make_response(jsonify(self.node_client.service_call(spliturl[1])))
                else:
                    return make_response(jsonify(services))

            if len(spliturl) > 0 and spliturl[0] == 'topics':
                topics = self.node_client.topics()
                if len(spliturl) > 1 and spliturl[1] in topics:
                    return make_response(jsonify(self.node_client.topic_extract(spliturl[1])))
                else:
                    return make_response(jsonify(topics))

            else:
                return make_response('', 404)


class BackEnd(restful.Resource):   # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/
    """
    View for backend pages
    """
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

        services = self.node_client.services()
        topics = self.node_client.topics()
        actions = [] #self.node_client.listacts()
        params = self.node_client.params()
        
        if path == CONFIG_PATH:
            cfg_resp = None
            dfile = definitions.manifest(services, topics, actions, full=full)
            if jsn:
                cfg_resp = make_response(str(dfile.tojson()), 200)
                cfg_resp.mimetype = 'application/json'
            else:
                cfg_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                cfg_resp.mimetype='text/plain'
            return cfg_resp

        if not suffix:
            if params is not None and path in params:
                msg = self.node_client.param_get(path)
            elif services is not None and path in services:
                msg = self.node_client.service_call(path)
            elif topics is not None and path in topics:
                if topics[path].get('allow_sub', None) is None:
                    self.logger.warn('405 : %s', path)
                    return make_response('', 405)

                msg = self.node_client.topic_extract(path)
            else:
                self.logger.warn('404 : %s', path)
                return make_response('', 404)

            #self.logger.debug('mimetypes : %s', request.accept_mimetypes)

            output_data = json.dumps(msg)
            mime_type = 'application/json'

            #if request_wants_ros(request):
            #    mime_type = ROS_MSG_MIMETYPE
            #    output_data = StringIO()
            #    if msg is not None:
            #        msg.serialize(output_data)
            #    output_data = output_data.getvalue()
            #else:  # we default to json
            #    # self.logger.debug('sending back json')
            #    mime_type = 'application/json'
            #    output_data = msgconv.extract_values(msg) if msg is not None else None
            #    output_data = json.dumps(output_data)

            response = make_response(output_data, 200)
            response.mimetype = mime_type
            return response

        path = path[:-(len(suffix) + 1)]
        sfx_resp = None
        if suffix == MSG_PATH and path in topics:
            sfx_resp = make_response(definitions.get_topic_msg(topics[path]), 200)
            sfx_resp.mimetype ='text/plain'
        elif suffix == SRV_PATH and path in self.ros_if.services:
            sfx_resp = make_response(definitions.get_service_srv(services[path]), 200)
            sfx_resp.mimetype ='text/plain'
        elif suffix == CONFIG_PATH:
            if path in services:
                service_name = path

                service = services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    sfx_resp = make_response(str(dfile.tojson()), 200)
                    sfx_resp.mimetype='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            elif path in topics:
                topic_name = path

                topic = topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    sfx_resp = make_response(str(dfile.tojson()), 200)
                    sfx_resp.mimetype ='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            else:
                sfx_resp = make_response('', 404)
        else:
            self.logger.warn('404 : %s', path)
            sfx_resp = make_response('', 404)
        return sfx_resp

    # TODO: think about login rest service before disabling REST services if not logged in
    def post(self, rosname):

        try:
            rosname = '/' + rosname
            #self.logger.debug('POST')
            length = int(request.environ['CONTENT_LENGTH'])
            use_ros = ('CONTENT_TYPE' in request.environ and
                       ROS_MSG_MIMETYPE == request.environ['CONTENT_TYPE'].split(';')[0].strip())

            services = self.node_client.services()
            topics = self.node_client.topics()
            params = self.node_client.params()

            if rosname in services:
                mode = 'service'
                service = services[rosname]
            elif rosname in topics:
                mode = 'topic'
                topic = topics[rosname]
                if topic.get('allow_pub', None) is None:
                    self.logger.warn('405 : %s', rosname)
                    return make_response('', 405)
            elif rosname in params:
                mode = 'param'
                param = params[rosname]
            else:
                self.logger.warn('404 : %s', rosname)
                return make_response('', 404)

            # we are now sending via the client node, which will convert the
            # received dict into the correct message type for the service (or
            # break, if it's wrong.)
            input_data = request.environ['wsgi.input'].read(length)
            input_data = json.loads(input_data or "{}")
            input_data.pop('_format', None)

            # input_msg = input_msg_type()  # was topic.rostype but we dont have it her ( cant serialize and transfer easily )
            # self.logger.debug('input_msg:%r', input_msg)
            # if use_ros:
            #     input_msg.deserialize(input_data)
            # else:
            #     input_data = json.loads(input_data)
            #     input_data.pop('_format', None)
            #     msgconv.populate_instance(input_data, input_msg)

            response = None
            if mode == 'service':
                self.logger.debug('calling service %s with msg : %s', service.get('name', None), input_data)
                ret_msg = self.node_client.service_call(rosname, input_data)

                if use_ros:
                    content_type = ROS_MSG_MIMETYPE
                    output_data = StringIO()
                    ret_msg.serialize(output_data)
                    output_data = output_data.getvalue()
                elif ret_msg:
                    output_data = ret_msg  # the returned message is already converted from ros format by the client
                    output_data['_format'] = 'ros'
                    output_data = json.dumps(output_data)
                    content_type = 'application/json'
                else:
                    output_data = "{}"
                    content_type = 'application/json'

                response = make_response(output_data, 200)
                response.mimetype = content_type

            elif mode == 'topic':
                self.logger.debug('publishing \n%s to topic %s', input_data, topic.get('name', None))
                self.node_client.topic_inject(rosname, input_data)
                response = make_response('{}', 200)
                response.mimetype = 'application/json'
            elif mode == 'param':
                self.logger.debug('setting \n%s param %s', input_data, param.get('name', None))
                self.node_client.param_set(rosname, input_data)
                response = make_response('{}', 200)
                response.mimetype = 'application/json'
            return response

        except Exception, e:
            self.logger.error('An exception occurred! => 500 %s', e)
            return make_response(e, 500)
