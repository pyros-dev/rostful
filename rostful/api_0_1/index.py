# -*- coding: utf-8 -*-
from __future__ import absolute_import

import importlib
import json
import base64
from flask import jsonify, url_for

import flask_restful as restful


from webargs.flaskparser import FlaskParser, use_kwargs
from webargs import fields

from . import api, current_app

parser = FlaskParser()

"""
Flask View Dealing with URLs
"""


@api.route('/', strict_slashes=False)
class Index(restful.Resource):  # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/

    """
    This is used especially to handle errors in this blueprint (REST style : JSON response)
    """
    def get(self):

        """Print available url and matching endpoints."""
        routes = {}
        for rule in current_app.url_map.iter_rules():
            try:
                if rule.endpoint != 'static':
                    if hasattr(current_app.view_functions[rule.endpoint], 'import_name'):
                        import_name = current_app.view_functions[rule.endpoint].import_name
                        obj = importlib.import_module(import_name)
                    else:
                        obj = current_app.view_functions[rule.endpoint]

                    routes[rule.rule] = {
                        "methods": ','.join(rule.methods),
                        "endpoint": rule.endpoint,
                        "description": obj.__doc__
                    }

            except Exception as exc:
                routes[rule.rule] = {
                        "methods": ','.join(rule.methods),
                        "endpoint": rule.endpoint,
                        "description": "INVALID ROUTE DEFINITION!!!"
                }
                route_info = "%s => %s" % (rule.rule, rule.endpoint)
                current_app.logger.error("Invalid route: %s" % route_info, exc_info=True)

        return jsonify(routes)

