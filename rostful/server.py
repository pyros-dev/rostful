# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import sys
import logging
import logging.handlers

try:
    from pyros import pyros_ctx, PyrosClient
except Exception, e:
    print "pyros module is not accessible in sys.path. It is required to run rostful."
    print "Exception caught : ", e
    print "sys.path = %r", sys.path
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
    # TODO : pass config file from command line here
    def __init__(self, testing=True):
        self.app = app

        if testing:
            self.app.config.from_object(config.Development)
        else:
            self.app.config.from_object(config.Testing)
        # TODO : flexible config by chosing file
        # TODO : flexible config by getting file from instance folder
        # TODO : flexible config by getting env var

    @property
    def logger(self):
        return self.app.logger

    def test_client(self, use_cookies=True):
        return self.app.test_client(use_cookies)

    def launch(self, host='127.0.0.1', port=8080, ros_args='', serv_type='flask', pyros_ctx_impl=None):
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
        print host, port

        # default to real module, if no other implementation passed as parameter (used for mock)
        pyros_ctx_impl = pyros_ctx_impl or pyros_ctx

        #One PyrosNode is needed for Flask.
        #TODO : check if still true with multiple web process
        with pyros_ctx_impl(name='rostful', argv=ros_args, base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..')) as node_ctx:
            set_pyros_client(node_ctx.client)

               # configure logger
            #if not debug:
            # add log handler for warnings and more to sys.stderr.
            #    self.logger.addHandler(logging.StreamHandler())
            #    self.logger.setLevel(logging.WARN)

            # adding file logging for everything to help debugging
            file_handler = logging.handlers.RotatingFileHandler('rostful.log', maxBytes=10000, backupCount=1)
            file_handler.setLevel(logging.INFO)
            self.app.logger.addHandler(file_handler)

            import socket  # just to catch the "Address already in use error"
            port_retries = 5
            while port_retries > 0:  # keep trying
                try:

                    if serv_type == 'flask':

                        log = logging.getLogger('werkzeug')
                        log.setLevel(logging.WARNING)

                        self.app.logger.info('Starting Flask server on port %d', port)
                        # debug is needed to investigate server errors.
                        # use_reloader set to False => killing the ros node also kills the server child.
                        self.app.run(
                            host=host,
                            port=port,
                            debug=True,
                            use_reloader=False,
                        )
                    elif serv_type == 'tornado':
                        self.app.logger.info('Starting Tornado server on port %d', port)
                        # enable_pretty_logging()  # enable this for debugging during development
                        http_server = HTTPServer(WSGIContainer(self.app))
                        http_server.listen(port)
                        IOLoop.instance().start()
                    # TODO : support more wsgi server setup : http://www.markjberger.com/flask-with-virtualenv-uwsgi-nginx/
                    break
                except socket.error, msg:
                    port_retries -= 1
                    port += 1
                    self.app.logger.error('Socket Error : {0}'.format(msg))


