### This package contains a Server ( instanciated once here to allow easy setup procedure, but not started )
from __future__ import absolute_import

import sys
import os

try:
    import pyros
except ImportError:
    # try again, in case we want to run from source and pyros is just there
    # usecase : running tests
    # TODO : make that better somehow... (needs pyros to detect pyros ???)
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'pyros', 'src')))
    import pyros

from . import config
from .server import Server

### TODO This package also contains a Client

__all__ = [
    'config',
    'Server',
]
