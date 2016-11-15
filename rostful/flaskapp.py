# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import logging
import logging.handlers

import errno

from . import config_template  # config template
from .context import register_teardown

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
# This should be in __init__ but ROS handling of python packages is breaking it.as
# TODO : move to __init__ if we ever get rid of ROS dependency

# external dependencies
from flask import Flask, url_for, redirect

# python package dependencies
import flask_cors as cors  # TODO : replace with https://github.com/may-day/wsgicors. seems more active.
from flask_reverse_proxy import FlaskReverseProxied


from .api_0_1 import api_blueprint as api_1_blueprint
from .api_0_2 import api_blueprint as api_2_blueprint
from .frontend import app_blueprint as frontend_blueprint


# Utility to dynamically create redirects (useful for bwcompat implementation)
def generate_redirect(endpoint, new_endpoint):
    def redirect_view():
        return redirect(url_for(endpoint, _external=True))  # external is needed to generate correct url behind a proxy
    redirect_view.__name__ = new_endpoint
    redirect_view.__doc__ = "Redirected URL route for " + endpoint
    return redirect_view


def create_app(configfile_override=None, logfile=None):

    app = Flask(
        'rostful',
        static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
        template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
        instance_relative_config=True
    )

    ### LOGGER SETUP ###

    # default log should show us all infos
    app.logger.setLevel(logging.DEBUG)

    # Setup logfile early
    if not logfile:
        logfile = os.path.join(app.instance_path, 'rostful.log')

    # adding file logging for everything to help debugging
    logdir = os.path.dirname(logfile)
    if not os.path.exists(logdir):
        try:
            os.makedirs(logdir)
        except OSError as exception:  # preventing race condition just in case
            if exception.errno != errno.EEXIST:
                raise

    # Adding a debug file logger
    file_handler = logging.handlers.RotatingFileHandler(
        logfile,
        maxBytes=100 * 131072,
        backupCount=10)
    file_handler.setLevel(logging.DEBUG)

    # create a formatter to have useful extra fields
    formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] [%(name)s] : %(message)s', )
    file_handler.setFormatter(formatter)

    app.logger.addHandler(file_handler)

    ### CONFIG SETUP ###

    # Config Workflow
    # 1. load hardcoded (in source) defaults, minimum expected to not break / crash.
    # 2. load config from default location (overriding hardcoded defaults).
    #    if not there, create it (at runtime, with the hardcoded default values)
    # 3. load user provided config from command arg if any, if missing file then except (user provided arg is wrong).
    #    if intent is to use file from default location, then no config arg should be provided.

    if not configfile_override:
        try:
            app.logger.info(
                "Loading config from {0}".format(os.path.join(app.instance_path, 'rostful.cfg')))


        except IOError as e:
            # If failed we create it from current default in package
            app.logger.warning("Cannot find rostful.cfg file to setup rostful. Creating it from default template...")
            with app.open_resource('config_template.py', 'r') as default_cfg_file:
                # Create instance config file name, to make it easy to modify when deploying
                filename = os.path.join(app.instance_path, 'rostful.cfg')
                if not os.path.isfile(filename):
                    # this will create the directories if needed
                    try:
                        os.makedirs(os.path.dirname(filename))
                    except OSError as exception:  # preventing race condition just in case
                        if exception.errno != errno.EEXIST:
                            raise
                with open(os.path.join(app.instance_path, 'rostful.cfg'), 'w') as instance_cfg_file:
                    for line in default_cfg_file:
                        instance_cfg_file.write(line)

            app.logger.warning("Configuration file created at {0}".format(os.path.join(app.instance_path, 'rostful.cfg')))

        # Attempting to set up default user configuration
        config_filepath = os.path.join(app.instance_path, 'rostful.cfg')
    else:
        config_filepath = configfile_override

    app.config.from_pyfile(config_filepath)

    # config can influence routes, so this needs to be done afterwards
    setup_app_routes(app)

    return app


def setup_app_routes(app):

    # Adding CORS middleware
    app.cors = cors.CORS(app, resources=r'/*', allow_headers='*')

    # Adding Reverse proxy middleware
    app.reverse_proxied = FlaskReverseProxied(app)

    # Temporary disabled until we can confirm if it s useful or not
    #
    # # REST API extended to render exceptions as json
    # # https://gist.github.com/grampajoe/6529609
    # # http://www.wiredmonk.me/error-handling-and-logging-in-flask-restful.html
    # class Api(restful.Api):
    #     def handle_error(self, e):
    #         # Attach the exception to itself so Flask-Restful's error handler
    #         # tries to render it.
    #         if not hasattr(e, 'data'):  # TODO : fix/improve this
    #             e.data = e
    #         return super(Api, self).handle_error(e)
    #
    # api = restful.Api(app)

    #
    # RESTful
    #
    app.logger.info('Registering api Blueprint on {0}'.format(app.config.get('BASEPATH', '/ros')))
    app.register_blueprint(api_1_blueprint, url_prefix=app.config.get('BASEPATH', '/ros'))

    # Building BWcompat routes with explicit redirects
    # TODO : double check, maybe not such a good idea ?
    #app.add_url_rule(app.config.get('BASEPATH', '/ros'), view_func=generate_redirect('api_0_1.backend', new_endpoint="_redirect_.ros._to_.api_0_1.backend"))

    # Next API : currently in development
    #app.logger.info('Registering api Blueprint on /api/v0.2_dev')
    #app.register_blueprint(api_2_blueprint, url_prefix='/api/v0.2_dev')

    # Usual Flask : This is not REST
    # Front Pages
    # self.app.add_url_rule('/favicon.ico', redirect_to=url_for('static', filename='favicon.ico'))
    # Follow pluggable views design : http://flask.pocoo.org/docs/0.10/views/
    # TODO : maybe add documentation/test pages from OPENAPI ?
    app.logger.info('Registering main Blueprint')
    app.register_blueprint(frontend_blueprint, url_prefix='/frontend')

    app.add_url_rule('/', view_func=generate_redirect('main.ros_list', new_endpoint="_redirect_._to_.ros_list"))

    app.logger.debug('Rostful Rules : {0}'.format(app.url_map))

    # Registering app context cleanup function
    register_teardown(app)


__all__ = [
    'app',
]
