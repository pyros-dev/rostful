# All ways to run rostful and all Manager commands
# should be defined here for consistency
from __future__ import absolute_import
import os
import sys

# importing current package if needed ( solving relative package import from __main__ problem )
if __package__ is None:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    # to be able to import tasks
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'src')))
    # absolute import not using __init__.py
    from gardener import gardener
else:
    # relative import using __init__.py
    from . import gardener

import logging
import click

#only needed to pass arguments to the worker

#TODO : handle config file via command line arg ?
#TODO : add more arguments to match celery worker useful options
@click.command()
@click.option('--ros_args', default='', help='holder for all ros arguments if needed')
def gardener_run(ros_args):
    #click.echo('ros_args=%s' % ros_args)
    try:
        logging.basicConfig(level=logging.INFO)

        logging.debug('Starting Gardener ')

        from rostful import rostful_celery_tasks
        #rostful_celery_tasks.add_together.apply_async((2,4))
        gardener.test_turtle_topics()
        #gardener.test_turtle_services()
        #gardener.test_turtle_action()


    except KeyboardInterrupt:
        logging.debug('Shutting down Gardener')



if __name__ == '__main__':
    gardener_run()
