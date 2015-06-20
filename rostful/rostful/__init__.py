### This package contains a Server ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

from . import celery_cfg as rostful_cfg_celery
from . import flask_cfg as rostful_cfg_flask

from .server import rostful_server
from . import celery_tasks as rostful_celery_tasks

### TODO This package also contains a Client

__all__ = [
    'rostful_cfg_flask',
    'rostful_cfg_celery',
    'rostful_server',
    'rostful_celery_tasks'
]
