# -*- coding: utf-8 -*-
from __future__ import absolute_import

import re
import sys

from flask import Flask, request, make_response, render_template, jsonify, redirect, views, url_for
import flask_restful as restful

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
from rostful import context

import time

import pyros

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'


def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH, SRV_PATH, MSG_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible

import simplejson
import logging
import logging.handlers
import tblib

from StringIO import StringIO


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


from flask import Flask, request, make_response, render_template, jsonify, redirect
from flask.views import MethodView
from flask_restful import reqparse
import flask_restful as restful

from . import app_blueprint, current_app


from webargs.flaskparser import FlaskParser, use_kwargs

parser = FlaskParser()

import urllib
from pyros import PyrosException
from rostful.exceptions import ServiceNotFound, ServiceTimeout, WrongMessageFormat, NoPyrosClient



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
# TODO: or probably best : https://github.com/rantav/flask-restful-swagger


@app_blueprint.route('/', strict_slashes=False, endpoint='ros_list')
def ros_list():
    current_app.logger.debug('in ros_list ')

    try:
        node_client = context.get_pyros_client()  # we retrieve pyros client from app context
        return render_template(
                'index.html',
                pathname2url=urllib.pathname2url,
                topics=node_client.topics(),
                services=node_client.services(),
                params=node_client.params(),
            )
    except NoPyrosClient as exc:
        # silently handle the case when we dont have pyros running
        return render_template(
            'index.html',
            pathname2url=urllib.pathname2url,
            topics=[],
            services=[],
            params=[],
        )
    except Exception:
        # failing request if unknown exception triggered
        raise



@app_blueprint.route('/<path:rosname>', strict_slashes=False, endpoint='ros_interface')
def ros_interface(rosname):
    current_app.logger.debug('in ros_interface with rosname: %r', rosname)

    node_client = context.get_pyros_client()  # we retrieve pyros client from app context

    # we might need to add "/" to rosname passed as url to match absolute service/topics names listed
    if not rosname.startswith("/"):
        rosname = "/" + rosname

    services = None
    topics = None
    with Timeout(30) as t:
        while not t.timed_out and (services is None or topics is None):
            try:
                services = node_client.services()
            except pyros.PyrosServiceTimeout:
                services = None
            try:
                topics = node_client.topics()
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
        raise ServiceNotFound("{0} not found among Pyros exposed services and topics".format(rosname))

