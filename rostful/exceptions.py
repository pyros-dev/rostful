# -*- coding: utf-8 -*-
from __future__ import absolute_import

# EXCEPTION CLASSES
# should be used to return anything that is not 2xx, python style.


class NoPyrosClient(Exception):
    status_code = 500

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv


class WrongMessageFormat(Exception):
    status_code = 400

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv


class ServiceTimeout(Exception):
    status_code = 504

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv


class ServiceNotFound(Exception):
    status_code = 404

    def __init__(self, message, status_code=None, traceback=None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.traceback = traceback

    def to_dict(self):
        rv = dict({})
        rv['message'] = self.message
        rv['traceback'] = self.traceback
        return rv
