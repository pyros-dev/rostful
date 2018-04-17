# -*- coding: utf-8 -*-
from __future__ import absolute_import

from ._version import __version__

import os

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
from .context import set_pyros_client, get_pyros_client

# HACK for ROS
from .flaskapp import create_app, setup_app_routes
from .exceptions import WrongMessageFormat, ServiceNotFound, ServiceTimeout, NoPyrosClient



### TODO This package also contains a Client

__all__ = [
    '__version__',
    'WrongMessageFormat',
    'ServiceNotFound',
    'ServiceTimeout',
    'NoPyrosClient',

    'create_app',
    'setup_app_routes',
    'set_pyros_client',
    'get_pyros_client',
]
