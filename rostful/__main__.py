#!/usr/bin/python
# All ways to run rostful and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys

import six

import click
import logging


# importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from rostful import create_app, set_pyros_client, setup_app_routes
else:
    from . import create_app, set_pyros_client, setup_app_routes


# Change that into something cleaner and related with the app itself (not server)
# Middleware ? app context ? Tricky with dynamic imports...
# middleware ref : https://github.com/miguelgrinberg/python-engineio/blob/master/engineio/middleware.py#L49
def pyros_start(config, ros_args='', pyros_ctx_impl=None):

    # implementing Config.get_namespace() for flask version < 1.0:
    namespace = 'PYROS_'
    trim_namespace = True
    lowercase = False
    rv = {}
    for k, v in six.iteritems(config):
        if not k.startswith(namespace):
            continue
        if trim_namespace:
            key = k[len(namespace):]
        else:
            key = k
        if lowercase:
            key = key.lower()
        rv[key] = v
    # rv should now contain a dictionary of namespaced key:value from config

    try:
        from pyros.server.ctx_server import pyros_ctx
        from pyros.client.client import PyrosClient
        #TMP until interface can be chosen otherwise...
        from pyros_interfaces_ros.pyros_ros import PyrosROS

    except Exception as e:
        logging.error("pyros module is not accessible in sys.path. It is required to run rostful.", exc_info=True)
        logging.error("sys.path = {0}".format(sys.path))
        raise

    # default to real module, if no other implementation passed as parameter (used for mock)
    pyros_ctx_impl = pyros_ctx_impl or pyros_ctx
    
    # very basic ROS choice. probably not the right way to get this ROS client to start the ROS node server.
    #  BUT we have no choice until pyros node can be launched directly...

    # One PyrosNode is needed for Flask.
    # TODO : check everything works as expected, even if the WSGI app is used by multiple processes

    logging.warning("ROSTFUL CONFIG {}".format(rv))

    try:
        node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args, node_impl=PyrosROS, pyros_config=rv)
    except TypeError as te:
        # bwcompat trick
        node_ctx_gen = pyros_ctx_impl(name='rostful', argv=ros_args, node_impl=PyrosROS,
                            base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    return node_ctx_gen

# TODO : handle ros arguments here
# http://click.pocoo.org/5/commands/#group-invocation-without-command
@click.group()
def cli():
    pass


#
# Arguments' default value is None here
# to use default values from config file if one is provided.
# If no config file is provided, internal defaults are used.
#
@cli.command()
@click.option('-h', '--host', default=None)
@click.option('-p', '--port', default=None)
@click.option('-s', '--server', default=None, type=click.Choice(['flask', 'tornado']))
@click.option('-c', '--config', default=None)  # this is the last possible config override, and has to be explicit.
@click.option('-l', '--logfile', default=None)  # this is the last possible logfile override, and has to be explicit.
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def run(host, port, server, config, logfile, ros_args):
    """
    Start rostful server.
    :param host: the local IP on which to serve rostful (0.0.0.0 for all)
    :param port: the local port on which to serve rostful
    :param server: the server to run our WSGI app (flask or tornado)
    :param config: the config file path, absolute, or relative to working directory
    :param logfile: the logfile path, absolute, or relative to working directory
    :param ros_args: the ros arguments (useful to absorb additional args when launched with roslaunch)
    TODO : get doctests to work here
    >>> run('127.0.0.1', '8888', False, 'flask', None, None)
    []
    >>> run('127.0.0.1', '8888', True, 'flask', None, None)
    >>> run('127.0.0.1', '4444', False, 'tornado', None, None)
    >>> run()

    """
    if port and isinstance(port, (str, unicode)):
        port = int(port)

    app = create_app(configfile_override=config, logfile=logfile)

    # Some logic for defaults value (might depend on config)
    server = server or app.config.get('SERVER_TYPE', 'tornado')

    app.logger.info(
        'rostful started with : host {host} port {port} config {config} logfile {logfile} ros_args {ros_args}'.format(
            host=host, port=port, config=config, logfile=logfile, ros_args=ros_args))

    # Starting pyros with latest config
    # TODO : move this out of here and let the user do it :
    # in ROS case : it should be done in launch file, and connect on user request (maybe even pass the zmq socket url)
    # in pyzmp case : it should be done dynamically, as much as possible outside of here.
    with pyros_start(config=app.config, ros_args=ros_args) as node_ctx:

        set_pyros_client(app, node_ctx.client)

        # configure logger

        # add log handler for warnings and more to sys.stderr.
        app.logger.addHandler(logging.StreamHandler())
        app.logger.setLevel(logging.WARN)

        import socket  # just to catch the "Address already in use" error
        port_retries = 5
        while port_retries > 0:  # keep trying
            try:
                # default server should be solid and production ready
                if server == 'flask':

                    log = logging.getLogger('werkzeug')
                    log.setLevel(logging.DEBUG)

                    app.logger.info('Starting Flask server on port {0}'.format(port))
                    # debug is needed to investigate server errors.
                    # use_reloader set to False => killing the ros node also kills the server child.
                    app.run(
                        host=host,
                        port=port,
                        debug=True,
                        use_reloader=False,
                    )
                elif server == 'tornado':

                    # Only import if needed
                    from tornado.wsgi import WSGIContainer
                    from tornado.httpserver import HTTPServer
                    from tornado.ioloop import IOLoop
                    from tornado.log import enable_pretty_logging

                    port = port or '5000'  # same default as flask
                    host = host or '127.0.0.1'  # same default as flask

                    app.logger.info('Starting Tornado server on {0}:{1}'.format(host, port))
                    # enable_pretty_logging()  # enable this for debugging during development
                    http_server = HTTPServer(WSGIContainer(app))
                    http_server.listen(port)
                    IOLoop.instance().start()
                # TODO : support more wsgi server setup : http://www.markjberger.com/flask-with-virtualenv-uwsgi-nginx/
                break
            except socket.error as msg:
                port_retries -= 1
                port += 1
                app.logger.error('Socket Error : {0}'.format(msg))


if __name__ == '__main__':
   cli()
