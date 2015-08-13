# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import sys
import logging

try:
    import rostful_node
except Exception, e:
    print "rostful_node module is not accessible in sys.path. It is required to run rostful."
    print "Exception caught : ", e
    print "sys.path = %r", sys.path
    raise

from . import flask_cfg

from flask import Flask, request, make_response, render_template, url_for, jsonify, redirect
import flask_security as security
import flask_cors as cors
import flask_restful as restful
import flask_login as login

from tornado.wsgi import WSGIContainer
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop

from . import db_models
from .db_models import db
from .flask_views import FrontEnd, BackEnd, Rostful


class Server(object):
    #TODO : pass config file from command line here
    def __init__(self):
        self.app = Flask('rostful',
                         static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
                         template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
                         instance_relative_config=True
                         )

        self.app.config.from_object(flask_cfg.Development)
        #TODO : flexible config by chosing file
        #TODO : flexible config by getting file from instance folder
        #TODO : flexible config by getting env var

        #initializes DB (needed here to allow migrations without launching flask server)
        db.init_app(self.app)
        self.db = db

        # Setup Flask-Security
        self.user_datastore = security.SQLAlchemyUserDatastore(self.db, db_models.User, db_models.Role)
        self.security = security.Security(self.app, self.user_datastore)

        # One of the simplest configurations. Exposes all resources matching /* to
        # CORS and allows the Content-Type header, which is necessary to POST JSON
        # cross origin.
        self.cors = cors.CORS(self.app, resources=r'/*', allow_headers='*')

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

        # TMP not sure which one is best
        self.api = restful.Api(self.app)
        #self.api = Api(self.app)

    @property
    def logger(self):
        return self.app.logger

    def _setup(self, ros_node, ros_node_client):
        self.ros_node = ros_node

        rostfront = FrontEnd.as_view('frontend', self.ros_node, self.logger)
        rostback = BackEnd.as_view('backend', self.ros_node, self.logger)
        rostful = Rostful.as_view('rostful', self.ros_node, self.logger)

        # self.app.add_url_rule('/favicon.ico', redirect_to=url_for('static', filename='favicon.ico'))
        # TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
        self.app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])

        #TODO : put everything under robot/worker name here ( so we can evolve to support multiple workers )
        self.app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])

        #TMP -> replace by using rosapi
        self.app.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
        self.app.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])

    def launch(self, host='127.0.0.1', port=8080, ros_args='', serv_type='flask'):

         print host, port

         #One RostfulNode is needed for Flask.
         #TODO : check if still true with multiple web process
         with rostful_node.RostfulCtx(name='rostful', argv=ros_args) as node_ctx:
             self._setup(node_ctx.node, node_ctx.client)

             import socket  # just to catch the "Address already in use error"
             port_retries = 5
             while port_retries > 0:  # keep trying
                 try:
                     if serv_type == 'flask':
                         rostful_server.app.logger.info('Starting Flask server on port %d', port)
                         # debug is needed to investigate server errors.
                         # use_reloader set to False => killing the ros node also kills the server child.
                         rostful_server.app.run(
                             host=host,
                             port=port,
                             debug=True,
                             use_reloader=False,
                         )
                     elif serv_type == 'tornado':
                         rostful_server.app.logger.info('Starting Tornado server on port %d', port)
                         http_server = HTTPServer(WSGIContainer(self.app))
                         http_server.listen(port)
                         IOLoop.instance().start()
                     break
                 except socket.error, msg:
                     port_retries -= 1
                     port += 1
                     rostful_server.app.logger.error('Socket Error : {0}'.format(msg))
                     #TODO: if port = default value, catch the "port already in use" exception and increment port number, and try again


# Creating THE only instance of Server.
rostful_server = Server()

# Setting up error handlers
@rostful_server.app.errorhandler(404)
def page_not_found(error):
    rostful_server.app.logger.error('Web Request ERROR 404 : %r', error)
    return render_template('error.html', error=error), 404

