# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import six
import sys
import logging
import logging.handlers

try:
    from pyros.server.ctx_server import pyros_ctx
    from pyros.client.client import PyrosClient
except Exception as e:
    logging.error("pyros module is not accessible in sys.path. It is required to run rostful.", exc_info=True)
    logging.error("sys.path = {0}".format(sys.path))
    raise

from . import config

# external dependencies
from flask import Flask, request, make_response, render_template, url_for, jsonify, redirect
from tornado.wsgi import WSGIContainer
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop
from tornado.log import enable_pretty_logging


from rostful import app, set_pyros_client


# TODO : move this into main. we probably dont need any specific server class here...
# TODO : check serving rostful with other web servers (nginx, etc.)
class Server(object):

    def __init__(self, config=None, logfile=None):
        self.app = app

        if logfile:
            # adding file logging for everything to help debugging
            logdir = os.path.dirname(logfile)
            if not os.path.exists(logdir):
                os.makedirs(logdir)
            file_handler = logging.handlers.RotatingFileHandler(logfile, maxBytes=10000, backupCount=1)
        else:
            # adding file logging for everything to help debugging
            if not os.path.exists(self.app.instance_path):
                os.makedirs(self.app.instance_path)
            file_handler = logging.handlers.RotatingFileHandler(os.path.join(self.app.instance_path, 'rostful.log'), maxBytes=10000, backupCount=1)

        file_handler.setLevel(logging.DEBUG)
        self.app.logger.addHandler(file_handler)

        if config:
            # if error we do need to raise and break here : the config file is not where the user expects it.
            self.app.config.from_pyfile(config)

    @property
    def logger(self):
        return self.app.logger

    def test_client(self, use_cookies=True):
        return self.app.test_client(use_cookies)

    # Default value should be secure and production ready to avoid unintentional bad setup.
    def launch(self, host=None, port=None, ros_args='', serv_type='tornado', pyros_ctx_impl=None):
        """
        Launch the current WSGI app in a web server (tornado or flask simple server)
        Will block until server is killed.

        :param host: the local ip to listen for connection on
        :param port: the local port to listen for connection on
        :param ros_args: the provided ros arguments that will be passed onto pyros node
        :param serv_type: the server type (tornado or flask)
        :param pyros_ctx_impl: the implementation of pyros context manager ( if different from normal module ).
                               This is useful for mocking it.
        :return: None
        """

        # default to real module, if no other implementation passed as parameter (used for mock)
        pyros_ctx_impl = pyros_ctx_impl or pyros_ctx

        if port and isinstance(port, (str, unicode)):
            port = int(port)

        # implementing Config.get_namespace() for flask version < 1.0:
        namespace = 'PYROS_'
        trim_namespace = True
        lowercase = False
        rv = {}
        for k, v in six.iteritems(self.app.config):
            if not k.startswith(namespace):
                continue
            if trim_namespace:
                key = k[len(namespace):]
            else:
                key = k
            if lowercase:
                key = key.lower()
            rv[key] = v
        # rv should now contain a dictionary of namespaced key:value from self.app.config

        # One PyrosNode is needed for Flask.
        # TODO : check everything works as expected, even if the WSGI app is used by multiple processes

        try:
            node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args, pyros_config=rv)
        except TypeError as te:
            # bwcompat trick
            node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args,
                                base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))

        with node_ctx_gen as node_ctx:
            set_pyros_client(node_ctx.client)

            # configure logger
            #if not debug:
            # add log handler for warnings and more to sys.stderr.
            #    self.logger.addHandler(logging.StreamHandler())
            #    self.logger.setLevel(logging.WARN)

            import socket  # just to catch the "Address already in use" error
            port_retries = 5
            while port_retries > 0:  # keep trying
                try:
                    # default server should be solid and production ready
                    serv_type = serv_type or self.app.config.get('SERVER_TYPE', 'tornado')
                    if serv_type == 'flask':

                        log = logging.getLogger('werkzeug')
                        log.setLevel(logging.DEBUG)

                        self.app.logger.info('Starting Flask server on port {0}'.format(port))
                        # debug is needed to investigate server errors.
                        # use_reloader set to False => killing the ros node also kills the server child.
                        self.app.run(
                            host=host,
                            port=port,
                            debug=True,
                            use_reloader=False,
                        )
                    elif serv_type == 'tornado':

                        port = port or '5000'  # same default as flask
                        host = host or '127.0.0.1'  # same default as flask

                        self.app.logger.info('Starting Tornado server on {0}:{1}'.format(host, port))
                        # enable_pretty_logging()  # enable this for debugging during development
                        http_server = HTTPServer(WSGIContainer(self.app))
                        http_server.listen(port)
                        IOLoop.instance().start()
                    # TODO : support more wsgi server setup : http://www.markjberger.com/flask-with-virtualenv-uwsgi-nginx/
                    break
                except socket.error as msg:
                    port_retries -= 1
                    port += 1
                    self.app.logger.error('Socket Error : {0}'.format(msg))


