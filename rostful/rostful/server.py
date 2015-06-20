# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import sys
import logging

try:
    import rostful_node
except:
    print "rostful_node module is not accessible in sys.path. It is required to run rostful."
    print "sys.path = %r", sys.path
    raise

from . import flask_cfg

from flask import Flask, request, make_response, render_template, jsonify, redirect
import flask_security as security
import flask_cors as cors
import flask_restful as restful
import flask_login as login
import flask_celery as celery

from celery.bin import Option

from . import db_models
from .db_models import db
from .flask_views import FrontEnd, BackEnd, Rostful
from .worker import RosArgs
from .celery_tasks import celery


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

        #Setup Flask-Celery
        self.celery = celery
        self.celery.init_app(self.app)

        self.celery.user_options['worker'].add(
            Option("--ros_args", action="store", dest="ros_args", default=None, help="Activate support of rapps")
        )
        self.celery.steps['worker'].add(RosArgs)

    def _setup(self, ros_node):
        self.ros_node = ros_node
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


    def launch_flask(self, host, port, ros_args):
        with rostful_node.RostfulNode(ros_args) as node:
            self._setup(node)

            # Adding a file logger if we are not in debug mode
            if not rostful_server.app.debug:
                file_handler = logging.RotatingFileHandler('rostful.log', maxBytes=10000, backupCount=1)
                file_handler.setLevel(logging.INFO)
                rostful_server.app.logger.addHandler(file_handler)

            rostful_server.app.logger.info('Starting Flask server on port %d', port)
            # debug is needed to investigate server errors.
            # use_reloader set to False => killing the ros node also kills the server child.
            rostful_server.app.run(host=host, port=port, debug=True, use_reloader=False)

    def launch_worker(self, ros_args):
        with rostful_node.RostfulNode(ros_args) as node:
            self._setup(node)

            #Hacking Flask-Celery Base Task
            task_base = self.celery.Task

            # Add Flask app context to celery instance.
            class RosNodeTask(task_base):
                """Task instance can access RosNode."""
                def __init__(self):
                    task_base.__init__(self)  # calling init the old way
                    self.ros_node = node

            setattr(RosNodeTask, 'abstract', True)
            setattr(self.celery, 'Task', RosNodeTask)

            #if isinstance(ros_args, list):
            #    ros_args
            # Starting Celery worker process
            rostful_server.celery.worker_main(argv=[
                'celery',
                #'--app=celery'
                #'--config=celery_cfg.Development',
                '--events',
                '--loglevel=DEBUG',
                #'--broker=' + celery_cfg.Development.CELERY_BROKER_URL,
                '--concurrency=1',
                '--autoreload',
                #'--ros_args=' + ros_args
            ], )


# Creating THE only instance of Server.
rostful_server = Server()

# Setting up error handlers
@rostful_server.app.errorhandler(404)
def page_not_found(error):
    rostful_server.app.logger.error('Web Request ERROR 404 : %r', error)
    return render_template('error.html', error=error), 404

