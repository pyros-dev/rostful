# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
from . import config

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
# This should be in __init__ but ROS handling of python packages is breaking it.as
# TODO : move to __init__ if we ever get rid of ROS dependency

# external dependencies
from flask import Flask

# python package dependencies
import flask_cors as cors  # TODO : replace with https://github.com/may-day/wsgicors. seems more active.

app = Flask(
    'rostful',
    static_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static'),
    template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'),
    instance_relative_config=True
)
# Implementing rostful default config as early as possible.
app.config.from_object(config)

# Adding CORS middleware
app.cors = cors.CORS(app, resources=r'/*', allow_headers='*')


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


# TODO : improve that into using app context.
# Creating pyros client should be simple and fast to be created everytime a request arrives.
# Pyros should also be able to support multiple client at the same time...

app.pyros_client = None


def set_pyros_client(pyros_client):
    app.pyros_client = pyros_client


def get_pyros_client():
    return app.pyros_client

# Following http://flask.pocoo.org/docs/0.10/patterns/packages/ with circular late import
# import rostful.flask_views


### TODO This package also contains a Client

__all__ = [
    'config',
    'app',
]
