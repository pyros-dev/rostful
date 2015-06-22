from __future__ import absolute_import

from celery import Celery, bootsteps
from celery.bin import Option


class RosArgs(bootsteps.StartStopStep):

    def __init__(self, worker, ros_args, **options):
        # store the ros config
        global g_ros_args
        g_ros_args = ros_args
        try:
            #self.ros_node = rostful_node.RostfulNode(g_ros_args)
            #self.ros_if = self.ros_node.ros_if
            #self.rocon_if = self.ros_node.rocon_if
            # List all requirement for this overseer to be able to start
            pass
        except Exception, e:
            raise



