# -*- coding: utf-8 -*-
from __future__ import absolute_import

from ._version import __version__

import os
from . import config

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/

# HACK for ROS
from .flaskapp import create_app
from .exceptions import WrongMessageFormat, ServiceNotFound, ServiceTimeout
from .context import set_pyros_client, get_pyros_client


### TODO This package also contains a Client

__all__ = [
    '__version__',
    'WrongMessageFormat',
    'ServiceNotFound',
    'ServiceTimeout',

    'config',
    'create_app',
    'set_pyros_client',
    'get_pyros_client',
]
