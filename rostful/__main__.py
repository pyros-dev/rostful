#!/usr/bin/python
# All ways to run rostful and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys
import click
import errno

#importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from rostful.server import Server
else:
    from .server import Server

import logging
from logging.handlers import RotatingFileHandler


@click.group()
def cli():
    pass


@cli.command()
def init():
    """
    Create missing configuration files.
    Useful just after install
    """
    # Start Server with default config
    rostful_server = Server()
    # Create instance config file name, to make it easy to modify when deploying
    filename = os.path.join(rostful_server.app.instance_path, 'rostful.cfg')
    if not os.path.isfile(filename) :
        #this will create the directories if needed
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as exception: #preventing race condition just in case
            if exception.errno != errno.EEXIST:
                raise
        #this will create the file
        rostful_server.app.open_instance_resource('rostful.cfg', 'w')


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
    """

    # Start Server with config passed as param
    rostful_server = Server(config, logfile)

    rostful_server.app.logger.info(
        'arguments passed : host {host} port {port} config {config} logfile {logfile} ros_args {ros_args}'.format(
            host=host, port=port, config=config, logfile=logfile, ros_args=ros_args))

    # Launch the server, potentially overriding host, port and server from config settings.
    rostful_server.launch(host, port, list(ros_args), server)

if __name__ == '__main__':
    cli()
