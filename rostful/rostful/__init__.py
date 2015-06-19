### This package contains a Server ( instanciated once here to allow easy setup procedure, but not started )

from .server import rostful_server
from .tasks import rostful_worker

### TODO This package also contains a Client




__all__ = ['rostful_server', 'rostful_worker']
