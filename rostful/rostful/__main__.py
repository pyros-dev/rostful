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
    from rostful import rostful_server
else:
    from . import rostful_server

import logging
from logging.handlers import RotatingFileHandler

# setup migrations
# python flask.py db migrate
# python flask.py db upgrade
# Need help ?
#python flask.py db --help

# TODO: use click to start the db migration (probably impossible)
# from flask_migrate import Migrate, MigrateCommand
# from flask_script import Manager
# migrate = Migrate(rostful_server.app, rostful_server.db)
# manager.add_command('db', MigrateCommand)


#TODO : http://stackoverflow.com/questions/29872867/using-flask-migrate-with-flask-script-and-application-factory/29882346#29882346
#manager.add_option("-c", "--config", dest="config_module", required=False)

#TODO : flexible config by command line param ???

@click.group()
def cli():
    pass

@cli.command()
def init():
    """
    Create useful configuration files and database on first install
    """
    #Create instance config file name, to make it easy to modify when deploying
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

    #run db upgrade to make sure our db schema is initialized ( here instead of in shell script )
    #TODO
    #add seed data to the database
    #TODO : prompt
    #prompt_bool(name, default=False, yes_choices=None, no_choices=None)
    rostful_server.user_datastore.create_user(email='admin@yujin.net', password='adminpass')
    rostful_server.db.session.commit()

@cli.command()
@click.option('-h', '--host', default='')
@click.option('-p', '--port', default=8000)
@click.option('-s', '--server_type', default='tornado', type=click.Choice(['flask', 'tornado']))
@click.option('ros_args', '-r', '--ros-arg', multiple=True, default='')
def run(host, port, server_type, ros_args):
    if isinstance(port, basestring):
        port = int(port)

    rostful_server.app.logger.info('host %r port %r', host, port)
    rostful_server.app.logger.info('ros_args %r', ros_args)

    #TODO : when called from python and no master found, do as roslaunch : create a master so it still can work from python
    #Launch the server
    rostful_server.launch(host, port, list(ros_args), server_type)

if __name__ == '__main__':
    cli()
