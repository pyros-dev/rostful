# -*- coding: utf-8 -*-
from __future__ import absolute_import

import os
import sys
import logging
import logging.handlers

##################
# TODO : remove this file : content moved to __init__
##################

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

# Follow pluggable views design : http://flask.pocoo.org/docs/0.10/views/
rostfront = FrontEnd.as_view('frontend', app.logger)
rostback = BackEnd.as_view('backend', app.logger)
rostful = Rostful.as_view('rostful', app.logger)

# self.app.add_url_rule('/favicon.ico', redirect_to=url_for('static', filename='favicon.ico'))
# TODO : improve with https://github.com/flask-restful/flask-restful/issues/429
app.add_url_rule('/', 'rostfront', view_func=rostfront, methods=['GET'])

app.add_url_rule('/<path:rosname>', 'rostfront', view_func=rostfront, methods=['GET'])
app.add_url_rule('/ros/<path:rosname>', 'rostback', view_func=rostback, methods=['GET', 'POST'])

# TODO : find a better way than reimplementing the thing here...
app.add_url_rule('/rostful', 'rostful', view_func=rostful, methods=['GET'])
app.add_url_rule('/rostful/<path:rostful_name>', 'rostful', view_func=rostful, methods=['GET'])

@app.errorhandler(404)
def page_not_found(self, error):
    self.app.logger.error('Web Request ERROR 404 : %r', error)
    return render_template('error.html', error=error), 404


def setup_pyros_client(pyros_client):
    app.pyros_client = pyros_client

# TODO : proper middleware + blueprint + app_context usage to connect to Pyros Client
# Ref : http://flask.pocoo.org/docs/0.10/patterns/appdispatch/#app-dispatch

