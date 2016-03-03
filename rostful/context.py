from __future__ import absolute_import
from flask import g

from rostful import app


def get_pyros_client():
    """
    Give access to pyros client from app context
    :return:
    """
    pyros_client = getattr(g, '_pyros_client', None)
    if pyros_client is None:
        pyros_client = g._pyros_client = app.pyros_client  # copying the app pyros client from app object to context
    return pyros_client


@app.teardown_appcontext
def teardown_pyros_client(exception):
    pyros_client = getattr(g, '_pyros_client', None)
    # TODO : cleanup if necessary.
    #if pyros_client is not None:
    #    pyros_client.close()