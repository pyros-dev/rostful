from __future__ import absolute_import
from flask import current_app, g

from .exceptions import NoPyrosClient

# TODO : improve that into using app context.
# Creating pyros client should be simple and fast to be created everytime a request arrives.
# Pyros should also be able to support multiple client at the same time...

def set_pyros_client(app, pyros_client):
    app.pyros_client = pyros_client


def get_pyros_client():
    """
    Give access to pyros client from app context
    :return:
    """
    pyros_client = getattr(g, '_pyros_client', None)
    if pyros_client is None:
        try:
            pyros_client = g._pyros_client = current_app.pyros_client  # copying the app pyros client from app object to context
        except AttributeError as exc:
            if not hasattr(current_app, 'pyros_client'):
                raise NoPyrosClient("Warning : Pyros Client not found in current app : {0}".format(exc))
            else:
                raise
    return pyros_client


def register_teardown(app):
    @app.teardown_appcontext
    def teardown_pyros_client(exception):
        pyros_client = getattr(g, '_pyros_client', None)
        # TODO : cleanup if necessary.
        #if pyros_client is not None:
        #    pyros_client.close()