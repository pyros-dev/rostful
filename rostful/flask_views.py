# -*- coding: utf-8 -*-
from __future__ import absolute_import

import re
import sys

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
from rostful import app
from rostful import context

import collections

import time

import pyros

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'


def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH, SRV_PATH, MSG_PATH, ACTION_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible

import json
import logging
import logging.handlers
import tblib

from StringIO import StringIO

from pyros.rosinterface import definitions
from pyros import PyrosServiceTimeout, PyrosServiceNotFound

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
    return re.search(r'(^|&)%s((=(true|1))|&|$)' % param_name, query_string, re.IGNORECASE)


from flask import Flask, request, make_response, render_template, jsonify, redirect
from flask.views import MethodView
from flask_restful import reqparse
import flask_restful as restful
import flask_login as login

from webargs.flaskparser import FlaskParser, use_kwargs

parser = FlaskParser()

import urllib
from pyros import PyrosException


### EXCEPTION CLASSES
# should be used to return anything that is not 2xx, python style.
class WrongMessageFormat(Exception):
    status_code = 400

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv

class ServiceTimeout(Exception):
    status_code = 504

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv

class ServiceNotFound(Exception):
    status_code = 404

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv


class Timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


"""
View for frontend pages
"""
# TODO: maybe consider http://www.flaskapi.org/
# TODO: or maybe better https://github.com/OAI/OpenAPI-Specification


class FrontEnd(MethodView):
    def __init__(self, logger):
        super(FrontEnd, self).__init__()
        self.logger = logger
        self.node_client = context.get_pyros_client()  # we retrieve pyros client from app context

    #TMP @login.login_required
    def get(self, rosname=None):
        self.logger.debug('in FrontEnd with rosname: %r', rosname)

        if self.node_client is None and not rosname:
            return render_template('index.html',
                                   pathname2url=urllib.pathname2url,
                                   topics=[],
                                   services=[],
                                   params=[],
            )
        elif self.node_client is not None and not rosname:
            return render_template('index.html',
                                   pathname2url=urllib.pathname2url,
                                   topics=self.node_client.topics(),
                                   services=self.node_client.services(),
                                   params=self.node_client.params(),
            )
        else:

            services = None
            topics = None
            with Timeout(30) as t:
                while not t.timed_out and (services is None or topics is None):
                    try:
                        services = self.node_client.services()
                    except pyros.PyrosServiceTimeout:
                        services = None
                    try:
                        topics = self.node_client.topics()
                    except pyros.PyrosServiceTimeout:
                        topics = None

            if t.timed_out:
                raise ServiceNotFound("Cannot list services and topics. No response from pyros.")

            if rosname in services:
                mode = 'service'
                service = services[rosname]
                return render_template('service.html', service=service)
            elif rosname in topics:
                mode = 'topic'
                topic = topics[rosname]
                return render_template('topic.html', topic=topic)
            else:
                raise ServiceNotFound("Cannot list services and topics. No response from pyros.")

class Rostful(restful.Resource):
    """
    Additional REST services provided by Rostful itself
    TMP : these should ideally be provided by a Ros node ( rostful-node ? RosAPI ? )
    """
    def __init__(self, logger):
        super(Rostful, self).__init__()
        self.logger = logger
        self.node_client = context.get_pyros_client()  # we retrieve pyros client from app context

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rostful_name=None):
        self.logger.debug('in Rostful with rostful_name: %r', rostful_name)
        if not rostful_name:
            return make_response(jsonify(name="Rostful",
                                         description="REST Services for ROS",
                                         version="v0.1"))
        else:
            spliturl = rostful_name.split('/')

            # fail early if no pyros client
            if self.node_client is None:
                return make_response('', 404)


            if len(spliturl) > 0 and spliturl[0] == 'services':
                services = self.node_client.services()
                if len(spliturl) > 1 and spliturl[1] in services:
                    svc_resp = None
                    with Timeout(30) as t:
                        while not t.timed_out and svc_resp is None:
                            try:
                                svc_resp = self.node_client.service_call(spliturl[1])
                            except pyros.PyrosServiceTimeout:
                                svc_resp = None
                    if t.timed_out or svc_resp is None:
                        raise ServiceNotFound("No response from pyros service interface.")
                    else:
                        return make_response(jsonify(svc_resp))
                else:
                    return make_response(jsonify(services))

            if len(spliturl) > 0 and spliturl[0] == 'topics':
                topics = self.node_client.topics()
                if len(spliturl) > 1 and spliturl[1] in topics:
                    tpc_resp = None
                    with Timeout(30) as t:
                        while not t.timed_out and tpc_resp is None:
                            try:
                                tpc_resp = self.node_client.topic_extract(spliturl[1])
                            except pyros.PyrosServiceTimeout:
                                tpc_resp = None
                    if t.timed_out or tpc_resp is None:
                        raise ServiceNotFound("No response from pyros topic interface.")
                    else:
                        return make_response(jsonify(tpc_resp))
                else:
                    return make_response(jsonify(topics))

            else:
                return make_response('', 404)


class BackEnd(restful.Resource):   # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/
    """
    View for backend pages
    """
    def __init__(self, logger):
        super(BackEnd, self).__init__()
        self.logger = logger
        self.node_client = context.get_pyros_client()  # we retrieve pyros client from app context

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

        # fail early if no pyros client
        if self.node_client is None:
            self.logger.warn('404 : %s', path)
            return make_response('', 404)

        services = self.node_client.services()
        topics = self.node_client.topics()
        params = self.node_client.params()
        
        if path == CONFIG_PATH:
            cfg_resp = None
            dfile = definitions.manifest(services, topics, full=full)
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
                msg = self.node_client.topic_extract(path)
            else:
                self.logger.warn('404 : %s', path)
                return make_response('', 404)

            #self.logger.debug('mimetypes : %s', request.accept_mimetypes)

            if msg is None:
                return make_response('', 204)  # returning no content if the message is not there
                # different than empty {} message

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

        # fail early if no pyros client
        if self.node_client is None:
            self.logger.warn('404 : %s', rosname)
            return make_response('', 404)

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
            elif rosname in params:
                mode = 'param'
                param = params[rosname]
            else:
                self.logger.warn('404 : %s', rosname)
                return make_response('', 404)

            # we are now sending via the client node, which will convert the
            # received dict into the correct message type for the service (or
            # break, if it's wrong.)

            # Trying to parse the input
            try:
                input_data = request.environ['wsgi.input'].read(length)
                input_data = json.loads(input_data or "{}")
                input_data.pop('_format', None)
                #TODO : We get the message structure via the topic, can we already use it for checking before calling rospy ?
            except ValueError as exc_value:
                raise WrongMessageFormat(
                    message="Your request payload was incorrect: {exc_value}".format(exc_value=exc_value),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )

            # input_msg = input_msg_type() # was topic.rostype but we dont have it here ( cant serialize and transfer easily )
            # self.logger.debug('input_msg:%r', input_msg)
            # if use_ros:
            #     input_msg.deserialize(input_data)
            # else:
            #     input_data = json.loads(input_data)
            #     input_data.pop('_format', None)
            #     msgconv.populate_instance(input_data, input_msg)

            response = None
            try:
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

            # converting pyros exceptions to proper rostful exceptions
            # except (InvalidMessageException, NonexistentFieldException, FieldTypeMismatchException) as exc_value:
            #     raise WrongMessageFormat(
            #         message=str(exc_value.message),
            #         traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
            #     )
            except PyrosServiceTimeout as exc_value:
                raise ServiceTimeout(
                    message=str(exc_value.message),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )
            except PyrosServiceNotFound as exc_value:
                raise ServiceNotFound(
                    message=str(exc_value.message),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )

        # returning local exceptions
        except WrongMessageFormat, wmf:
            self.logger.error('Wrong message format! => {status} \n{exc}'.format(
                status=wmf.status_code,
                exc=wmf.message
            ))
            return make_response(json.dumps(wmf.to_dict()), wmf.status_code)

        except ServiceTimeout, st:
            self.logger.error('Service Timeout! => {status} \n{exc}'.format(
                status=st.status_code,
                exc=st.message
            ))
            return make_response(json.dumps(st.to_dict()), st.status_code)

        except ServiceNotFound, snf:
            self.logger.error('Service Not Found! => {status} \n{exc}'.format(
                status=snf.status_code,
                exc=snf.message
            ))
            return make_response(json.dumps(snf.to_dict()), snf.status_code)

        # Generic way to return Exceptions we don't know how to handle
        # But we can do a tiny bit better if it s a PyrosException
        except Exception as exc_value:
            exc_type, exc_value, tb = sys.exc_info()
            tb = tblib.Traceback(tb)
            exc_dict = {
                    'exc_type': str(exc_type),
                    # TODO : it would be nice if pyros exception wouldnt need such a check...
                    'exc_value': str(exc_value.message) if isinstance(exc_value, PyrosException) else str(exc_value),
                    'traceback': tb.to_dict()
                 }
            self.logger.error('An exception occurred! => 500 \n{exc}'.format(exc=exc_dict))
            return make_response(json.dumps(exc_dict), 500)
            # return make_response(e, 500)


### Setting up routes here for now

# Follow pluggable views design : http://flask.pocoo.org/docs/0.10/views/
rostfront = FrontEnd.as_view('frontend', app.logger)
rostback = BackEnd.as_view('backend', app.logger)
rostful = Rostful.as_view('rostful', app.logger)

# self.app.add_url_rule('/favicon.ico', redirect_to=url_for('static', filename='favicon.ico'))
# TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])

app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])

# TODO : find a better way than reimplementing the thing here...
app.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
app.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])