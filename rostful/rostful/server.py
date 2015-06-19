# -*- coding: utf-8 -*-
from __future__ import absolute_import
from rostful_node import RostfulNode

import os

from flask import Flask, request, make_response, render_template, jsonify, redirect
import flask_security as security
import flask_cors as cors
import flask_restful as restful
import flask_login as login

from . import db_models
from .db_models import db
from .flask_views import FrontEnd, BackEnd, Rostful


import signal
import sys
def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)


class Server(object):
    #TODO : pass config file from command line here
    def __init__(self):
        #TODO : change into application factory (https://github.com/miguelgrinberg/Flask-Migrate/issues/45)
        #because apparently ROS start python node from ~user/.ros, and it obviously cant find templates there
        self.app = Flask('rostful',
                         static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
                         template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
                         instance_relative_config=True
                         )

        self.app.config.from_object('rostful.cfg.flask.Development')
        #TODO : flexible config by chosing file
        #TODO : flexible config by getting file from instance folder
        #TODO : flexible config by getting env var

        #initializes DB
        db.init_app(self.app)
        self.db = db

        # Setup Flask-Security
        self.user_datastore = security.SQLAlchemyUserDatastore(self.db, db_models.User, db_models.Role)
        self.security = security.Security(self.app, self.user_datastore)

        # One of the simplest configurations. Exposes all resources matching /* to
        # CORS and allows the Content-Type header, which is necessary to POST JSON
        # cross origin.
        self.cors = cors.CORS(self.app, resources=r'/*', allow_headers='Content-Type')

    def launch(self, ros_args):



        self.ros_node = RostfulNode(ros_args)

        import rospy

        def handler(signal, frame):
            print('You pressed Ctrl+C!')
            rospy.signal_shutdown('Closing')
            sys.exit(0)
            #self.shutdown()

        signal.signal(signal.SIGINT, handler)


        rostfront = FrontEnd.as_view('frontend', self.ros_node)
        rostback = BackEnd.as_view('backend', self.ros_node)
        rostful = Rostful.as_view('rostful', self.ros_node)

        # TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
        self.app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
        self.app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])
        self.app.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
        self.app.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])
        self.api = restful.Api(self.app)

    def shutdown(self):
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()


# Creating THE only instance of Server.
rostful_server = Server()

# Setting up error handlers
@rostful_server.app.errorhandler(404)
def page_not_found(error):
    rostful_server.app.logger.error('Web Request ERROR 404 : %r', error)
    return render_template('error.html', error=error), 404

