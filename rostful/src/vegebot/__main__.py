# All ways to run rostful and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys

# importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    # absolute import not using __init__.py
    from celery_worker import tasks
else:
    # relative import using __init__.py
    from . import tasks

import logging
import click

#only needed to pass arguments to the worker
import config.default
import config.development

#TODO : handle config file via command line arg ?
#TODO : add more arguments to match celery worker useful options
@click.command()
@click.option('--ros_args', default='', help='holder for all ros arguments if needed')
def worker(ros_args):
    #click.echo('ros_args=%s' % ros_args)
    try:
        logging.basicConfig(level=logging.INFO)

        logging.debug('Starting Celery worker')
        # Starting Celery worker process
        tasks.celery.worker_main(argv=['celery', '--loglevel=DEBUG', '--broker=' + config.default.CELERY_BROKER_URL, '--autoreload', '--ros_args=' + ros_args], )

    except KeyboardInterrupt:
        logging.debug('Shutting down the Celery worker')



if __name__ == '__main__':
    worker()
