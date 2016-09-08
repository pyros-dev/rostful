# -*- coding: utf-8 -*-
from __future__ import absolute_import

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
import types
from flask import Blueprint, current_app
import flask_restful

app_blueprint = Blueprint('main', __name__)

# importing package content mandatory to setup routes for blueprint
from . import flask_views

"""
The Package managing api
"""

__all__ = [
    'app_blueprint',
    'current_app',

]
