# -*- coding: utf-8 -*-
from __future__ import absolute_import

import re
import sys

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
from rostful import get_pyros_client

import time

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'


def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH, SRV_PATH, MSG_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible

import simplejson
import tblib

try:
    # Python 2
    from cStringIO import StringIO
except ImportError:
    # Python 3
    from io import StringIO


ROS_MSG_MIMETYPE = 'application/vnd.ros.msg'
def ROS_MSG_MIMETYPE_WITH_TYPE(rostype):
    if isinstance(rostype, type):
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


from flask import request, make_response, render_template, jsonify, redirect
from flask.views import MethodView
from flask_restful import reqparse
import flask_restful as restful

from . import api, api_blueprint, current_app

from webargs.flaskparser import FlaskParser, use_kwargs

parser = FlaskParser()

from pyros_common.exceptions import PyrosException

from rostful.exceptions import ServiceNotFound, ServiceTimeout, WrongMessageFormat


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


# TODO : check if we can simplify this by dynamically generating usual simple flask route (one for each service/topic)
# this way we could use url_for simply and rely on flask url build scheme...
#@api.resource('/', '/<path:rosname>', strict_slashes=False)
# TO FIX flask restful problem
# TODO : get rid of flask restful dependency, it is apparently not maintained any longer
# @api.route('/')
# @api.route('/<path:rosname>')
class BackEnd(restful.Resource):   # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/
    """
    View for backend pages
    """
    def __init__(self, rosname=None):
        super(BackEnd, self).__init__()

        self.node_client = get_pyros_client()  # we retrieve pyros client from app context

        # dynamic import
        from pyros_interfaces_ros import definitions
        from pyros.client.client import PyrosServiceTimeout, PyrosServiceNotFound

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rosname=None):
        current_app.logger.debug('in BackEnd with rosname: %r', rosname)

        # dynamic import
        from pyros_interfaces_ros import definitions
        from pyros.client.client import PyrosServiceTimeout, PyrosServiceNotFound

        # TODO : replace this with webargs ( less buggy )
        parser = reqparse.RequestParser()
        # somehow this breaks requests now... disabling for now.
        # parser.add_argument('full', type=bool)
        # parser.add_argument('json', type=bool)
        args = parser.parse_args()

        path = '/' + (rosname or '')
        full = args.get('full', True)
        jsn = args.get('json', True)

        suffix = get_suffix(path)

        # fail early if no pyros client
        if self.node_client is None:
            current_app.logger.warn('404 : %s', path)
            return make_response('', 404)

        services = self.node_client.services()
        topics = self.node_client.topics()
        params = self.node_client.params()

        # Handling special case empty rosname and suffix
        if path == '/' and not suffix:
            return jsonify({
                "services": services,
                "topics": topics,
                "params": params
            })

        # special case to get rosdef of all services and topics
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
                current_app.logger.warn('404 : %s', path)
                return make_response('', 404)

            #current_app.logger.debug('mimetypes : %s', request.accept_mimetypes)

            if msg is None:
                return make_response('', 204)  # returning no content if the message is not there
                # different than empty {} message

            # This changes nan to null (instead of default json encoder that changes nan to invalid json NaN).
            # some client might be picky and this should probably be a configuration setting...
            output_data = simplejson.dumps(msg, ignore_nan=True)
            mime_type = 'application/json'

            #if request_wants_ros(request):
            #    mime_type = ROS_MSG_MIMETYPE
            #    output_data = StringIO()
            #    if msg is not None:
            #        msg.serialize(output_data)
            #    output_data = output_data.getvalue()
            #else:  # we default to json
            #    # current_app.logger.debug('sending back json')
            #    mime_type = 'application/json'
            #    output_data = msgconv.extract_values(msg) if msg is not None else None
            #    output_data = json.dumps(output_data)

            response = make_response(output_data, 200)
            response.mimetype = mime_type
            return response

        path = path[:-(len(suffix) + 1)]
        sfx_resp = None
        if suffix == MSG_PATH and path in topics:
            if jsn:
                # TODO : find a better way to interface here...
                sfx_resp = make_response(simplejson.dumps(topics[path].get('msgtype')), 200)
                sfx_resp.mimetype = 'application/json'
            else:
                # broken now, cannot access pyros.rosinterface.topic.get_topic_msg
                # TODO : check if we still need that feature ? JSON first -> maybe not...
                # Or we do it in another way (without needing to import that module)
                #sfx_resp = make_response(definitions.get_topic_msg(topics[path]), 200)
                sfx_resp = make_response('Deprecated feature. use _rosdef instead', 200)
                sfx_resp.mimetype = 'text/plain'
                pass
        elif suffix == SRV_PATH and path in services:
            if jsn:
                sfx_resp = make_response(simplejson.dumps(services[path].get('srvtype'), ignore_nan=True), 200)
                sfx_resp.mimetype = 'application/json'
            else:
                # broken now, cannot access pyros.rosinterface.service.get_service_srv
                # TODO : check if we still need that feature ? JSON first -> maybe not...
                # Or we do it in another way (without needing to import that module)
                #sfx_resp = make_response(definitions.get_service_srv(services[path]), 200)
                sfx_resp = make_response('Deprecated feature. use _rosdef instead', 200)
                sfx_resp.mimetype = 'text/plain'
        elif suffix == CONFIG_PATH:
            if path in services:
                service_name = path

                service = services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    sfx_resp = make_response(simplejson.dumps(dfile.tojson(), ignore_nan=True), 200)
                    sfx_resp.mimetype='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            elif path in topics:
                topic_name = path

                topic = topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    sfx_resp = make_response(simplejson.dumps(dfile.tojson(), ignore_nan=True), 200)
                    sfx_resp.mimetype ='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            else:
                sfx_resp = make_response('', 404)
        else:
            current_app.logger.warn('404 : %s', path)
            sfx_resp = make_response('', 404)
        return sfx_resp

    # TODO: think about login rest service before disabling REST services if not logged in
    def post(self, rosname, *args, **kwargs):

        # fail early if no pyros client
        if self.node_client is None:
            current_app.logger.warn('404 : %s', rosname)
            return make_response('', 404)

        try:
            rosname = '/' + rosname
            #current_app.logger.debug('POST')
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
                current_app.logger.warn('404 : %s', rosname)
                return make_response('', 404)

            # we are now sending via the client node, which will convert the
            # received dict into the correct message type for the service (or
            # break, if it's wrong.)

            # Trying to parse the input
            try:
                input_data = request.environ['wsgi.input'].read(length)
                input_data = simplejson.loads(input_data or "{}")
                input_data.pop('_format', None)
                #TODO : We get the message structure via the topic, can we already use it for checking before calling rospy ?
            except ValueError as exc_value:
                raise WrongMessageFormat(
                    message="Your request payload was incorrect: {exc_value}".format(exc_value=exc_value),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )

            # input_msg = input_msg_type() # was topic.rostype but we dont have it here ( cant serialize and transfer easily )
            # current_app.logger.debug('input_msg:%r', input_msg)
            # if use_ros:
            #     input_msg.deserialize(input_data)
            # else:
            #     input_data = json.loads(input_data)
            #     input_data.pop('_format', None)
            #     msgconv.populate_instance(input_data, input_msg)

            response = None
            try:
                if mode == 'service':
                    current_app.logger.debug('calling service %s with msg : %s', service.get('name'), input_data)
                    ret_msg = self.node_client.service_call(rosname, input_data)

                    if use_ros:
                        content_type = ROS_MSG_MIMETYPE
                        output_data = StringIO()
                        ret_msg.serialize(output_data)
                        output_data = output_data.getvalue()
                    elif ret_msg:
                        output_data = ret_msg  # the returned message is already converted from ros format by the client
                        output_data['_format'] = 'ros'
                        output_data = simplejson.dumps(output_data, ignore_nan=True)
                        content_type = 'application/json'
                    else:
                        output_data = "{}"
                        content_type = 'application/json'

                    response = make_response(output_data, 200)
                    response.mimetype = content_type

                elif mode == 'topic':
                    current_app.logger.debug('publishing \n%s to topic %s', input_data, topic.get('name'))
                    self.node_client.topic_inject(rosname, input_data)
                    response = make_response('{}', 200)
                    response.mimetype = 'application/json'
                elif mode == 'param':
                    current_app.logger.debug('setting \n%s param %s', input_data, param.get('name'))
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
        except WrongMessageFormat as wmf:
            current_app.logger.error('Wrong message format! => {status} \n{exc}'.format(
                status=wmf.status_code,
                exc=wmf.message
            ))
            return make_response(simplejson.dumps(wmf.to_dict(), ignore_nan=True), wmf.status_code)

        except ServiceTimeout as st:
            current_app.logger.error('Service Timeout! => {status} \n{exc}'.format(
                status=st.status_code,
                exc=st.message
            ))
            return make_response(simplejson.dumps(st.to_dict(), ignore_nan=True), st.status_code)

        except ServiceNotFound as snf:
            current_app.logger.error('Service Not Found! => {status} \n{exc}'.format(
                status=snf.status_code,
                exc=snf.message
            ))
            return make_response(simplejson.dumps(snf.to_dict(), ignore_nan=True), snf.status_code)

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
            current_app.logger.error('An exception occurred! => 500 \n{exc}'.format(exc=exc_dict))
            return make_response(simplejson.dumps(exc_dict, ignore_nan=True), 500)
            # return make_response(e, 500)

api.add_resource(BackEnd, '/','/<path:rosname>')
# TO have more than json representation : http://stackoverflow.com/a/28520065
