from __future__ import absolute_import

from celery import Celery, bootsteps
from celery.bin import Option


class RosArgs(bootsteps.StartStopStep):

    def __init__(self, worker, ros_args, **options):
        print('Called when the WorkController instance is constructed')
        print('Arguments to WorkController: {0!r}'.format(options))
        # store the ros config
        global g_ros_args
        g_ros_args = ros_args
        #worker.ros_args = ros_args
        try:
            #self.ros_node = rostful_node.RostfulNode(g_ros_args)
            #self.ros_if = self.ros_node.ros_if
            #self.rocon_if = self.ros_node.rocon_if
            # List all requirement for this overseer to be able to start
            pass
        except Exception, e:
            raise

    def create(self, worker):
        # this method can be used to delegate the action methods
        # to another object that implements ``start`` and ``stop``.
        return self

    def start(self, worker):
        print('Called when the worker is started.')

    def stop(self, worker):
        print("Called when the worker shuts down.")

    def terminate(self, worker):
        print("Called when the worker terminates")
