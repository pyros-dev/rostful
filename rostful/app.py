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

# python package dependencies
import flask_cors as cors  # TODO : replace with https://github.com/may-day/wsgicors. seems more active.
import flask_restful as restful
import flask_security as security

from . import db_models
from .db_models import db
from .flask_views import FrontEnd, BackEnd, Rostful

app = Flask(
    'rostful',
    static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
    template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
    instance_relative_config=True
)

# initializes DB (needed here to allow migrations without launching flask server)
db.init_app(app)

# Adding CORS middleware
app.cors = cors.CORS(app, resources=r'/*', allow_headers='*')

# Setup Flask-Security
user_datastore = security.SQLAlchemyUserDatastore(db, db_models.User, db_models.Role)
security = security.Security(app, user_datastore)


# REST API extended to render exceptions as json
# https://gist.github.com/grampajoe/6529609
# http://www.wiredmonk.me/error-handling-and-logging-in-flask-restful.html
class Api(restful.Api):
    def handle_error(self, e):
        # Attach the exception to itself so Flask-Restful's error handler
        # tries to render it.
        if not hasattr(e, 'data'):  # TODO : fix/improve this
            e.data = e
        return super(Api, self).handle_error(e)

api = restful.Api(app)


# Setting up error handlers
# TODO : HOW ??
@app.errorhandler(404)
def page_not_found(error):
    app.logger.error('Web Request ERROR 404 : %r', error)
    return render_template('error.html', error=error), 404


def setup_pyros_client(self, ros_node_client, debug=False):
    rostfront = FrontEnd.as_view('frontend', ros_node_client, self.logger, debug)
    rostback = BackEnd.as_view('backend', ros_node_client, self.logger, debug)
    rostful = Rostful.as_view('rostful', ros_node_client, self.logger, debug)

    # self.app.add_url_rule('/favicon.ico', redirect_to=url_for('static', filename='favicon.ico'))
    # TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
    self.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])

    # TODO : put everything under robot/worker name here ( so we can evolve to support multiple workers )
    self.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
    self.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])

    # TMP -> replace by using rosapi
    self.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
    self.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])


# FIXME : currently patching app ( waiting for proper middleware implementation )
import types
app.setup_pyros_client = types.MethodType(setup_pyros_client, app)


# TODO : extract WSGI app from this. server and WSGI app sould be separate (allows separate testing)
class Server(object):
    # TODO : pass config file from command line here
    def __init__(self, testing=True):

        if testing:
            self.app.config.from_object(config.Development)
        else:
            self.app.config.from_object(config.Testing)
        # TODO : flexible config by chosing file
        # TODO : flexible config by getting file from instance folder
        # TODO : flexible config by getting env var

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
            self._setup(node_ctx.client, False if serv_type == 'tornado' else True)

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

