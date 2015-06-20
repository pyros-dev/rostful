### This package contains a Server ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

from .server import rostful_server
from .tasks import rostful_worker

### TODO This package also contains a Client




__all__ = [
    'rostful_cfg_flask',
    'rostful_cfg_celery',
    'rostful_server',
    'rostful_worker'
]
