from __future__ import absolute_import

from celery import Celery, Task, bootsteps
from celery.bin import Option

class RosArgs(bootsteps.StartStopStep):

    def __init__(self, worker, ros_args, **options):
        # store the ros config
        global g_ros_args
        g_ros_args = ros_args.split()
        try:
            self.ros_node = RostfulNode(g_ros_args)
            self.ros_if = self.ros_node.ros_if
            self.rocon_if = self.ros_node.rocon_if
            # List all requirement for this overseer to be able to start
            pass
        except Exception, e:
            raise

class Worker(object):
    #TODO : pass config file from command line here
    def __init__(self):
        self.app = Celery()  # Celery(broker=cfg.celery.Development.CELERY_BROKER_URL, backend=cfg.celery.Development.CELERY_RESULT_BACKEND)
        self.app.config_from_object('rostful.cfg.celery.Development')
        self.app.user_options['worker'].add(
            Option("--ros_args", action="store", dest="ros_args", default=None, help="Activate support of rapps")
        )
        self.app.steps['worker'].add(RosArgs)

    def launch(self, ros_args):
        # Starting Celery worker process
        self.app.worker_main(argv=[
            'celery',
            # '--config=',
            '--events',
            '--loglevel=DEBUG',
            #'--broker=' + cfg.celery.Development.CELERY_BROKER_URL,
            '--concurrency=1',
            '--autoreload',
            '--ros_args=' + ros_args], )
        # TODO : replace all these args with celery settings
        pass

    def shutdown(self):
        # TODO
        pass

rostful_worker = Worker()


