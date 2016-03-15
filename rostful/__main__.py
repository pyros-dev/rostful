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
    from rostful import Server
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
    Create useful configuration files and database on first install
    """
    # Start Server with default config
    rostful_server = Server()
    # Create instance config file name, to make it easy to modify when deploying
    filename = os.path.join(rostful_server.app.instance_path, 'flask_config.py')
    if not os.path.isfile(filename) :
        #this will create the directories if needed
        try:
            os.makedirs(os.path.dirname(filename))
        except OSError as exception: #preventing race condition just in case
            if exception.errno != errno.EEXIST:
                raise
        #this will create the file
        rostful_server.app.open_instance_resource('flask_config.py', 'w')


@cli.command()
@click.option('-h', '--host', default='')
@click.option('-p', '--port', default=8000)
@click.option('-s', '--server_type', default='tornado', type=click.Choice(['flask', 'tornado']))
@click.option('-c', '--config', default='rostful.config')
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def run(host, port, server_type, config, ros_args):
    if isinstance(port, basestring):
        port = int(port)

    # Start Server with config passed as param
    rostful_server = Server(config)

    rostful_server.app.logger.info('host %r port %r', host, port)
    rostful_server.app.logger.info('config %r', config)
    rostful_server.app.logger.info('ros_args %r', ros_args)

    #TODO : when called from python and no master found, do as roslaunch : create a master so it still can work from python
    #Launch the server
    rostful_server.launch(host, port, list(ros_args), server_type)

if __name__ == '__main__':
    cli()
