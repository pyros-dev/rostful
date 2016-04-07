# -*- coding: utf-8 -*-
from __future__ import absolute_import

from ._version import __version__

import os
from . import config

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/

# HACK for ROS
from .flaskapp import app


# TODO : improve that into using app context.
# Creating pyros client should be simple and fast to be created everytime a request arrives.
# Pyros should also be able to support multiple client at the same time...

app.pyros_client = None


def set_pyros_client(pyros_client):
    app.pyros_client = pyros_client


def get_pyros_client():
    return app.pyros_client

# Following http://flask.pocoo.org/docs/0.10/patterns/packages/ with circular late import
from rostful.flask_views import WrongMessageFormat, ServiceNotFound, ServiceTimeout


### TODO This package also contains a Client

__all__ = [
    '__version__',
    'WrongMessageFormat',
    'ServiceNotFound',
    'ServiceTimeout',

    'config',
    'app',
    'set_pyros_client',
]
