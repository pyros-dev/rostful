# -*- coding: utf-8 -*-
from __future__ import absolute_import, print_function

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
import types
from flask import Blueprint, current_app
import flask_restful

api_blueprint = Blueprint('api_0_2', __name__)


api = flask_restful.Api(api_blueprint)


# TODO get rid of flask-restful dependency here (seems no longer maintained), and implement functionality directly with flask.
# Might be just slowto catch up on flask, but better remove it anyway (low benefit for the cost of maintenance)
# Try FlaskAPI...

# TODO : double check if needed or already in flask-restful
# def api_route(self, *args, **kwargs):
#     def wrapper(cls):
#         self.add_resource(cls, *args, **kwargs)
#         return cls
#     return wrapper

# api.route = types.MethodType(api_route, api)

# importing package content mandatory to setup routes for blueprint
#from . import flask_views

"""
The Package managing api
"""

__all__ = [
    'api_blueprint',
    'api',
    'current_app'
]
