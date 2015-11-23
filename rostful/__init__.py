### This package contains a Server ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

from . import flask_cfg as rostful_cfg_flask

from .server import rostful_server

### TODO This package also contains a Client

__all__ = [
    'rostful_cfg_flask',
    'rostful_server',
]
