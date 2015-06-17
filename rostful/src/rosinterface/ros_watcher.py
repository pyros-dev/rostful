# -*- coding: utf-8 -*-
from __future__ import absolute_import
from concurrent import futures
import threading
import socket
import rospy

#This module does not depend on rocon, just use connections from the package if available
_ROCON = False
try:
    from rocon_python_comms import connections, PUBLISHER, SUBSCRIBER, SERVICE, ACTION_SERVER, ACTION_CLIENT
    _ROCON = True
except Exception, e:
    import rosgraph


class ROSWatcher(threading.Thread):  #TODO : DO NOT inherit from thread. instead use the executor for watching.

    def __init__(self, topics_change_cb, services_change_cb, actions_change_cb):  # TODO : use Queue for callbacks to be executed in main thread.
        super(ROSWatcher, self).__init__()

        self.executor = futures.ThreadPoolExecutor(max_workers=1)

        #TODO : acessing members should be done via property that copy the object to avoid accidental modification during use

        if _ROCON:
            #current services topics and actions available
            self.connections = connections.ConnectionCache()
            # TODO : improve this by using rosapi or any specific node that broadcast these on a latched publisher, to avoid annoying the master everytime
        else:
            # in bare ROS, our only option is to query the master
            # TODO : improve : add a dependency to a package/node that provides list of topics/services as a latched topic
            self._master = rosgraph.Master(rospy.get_name())
            self.publishers = []
            self.subscribers = []
            self.services = []
        self.topics_change_cb = topics_change_cb
        self.services_change_cb = services_change_cb
        self.actions_change_cb = actions_change_cb

    def run(self):
        """
        Starting this thread asynchronously
        """
        rate = rospy.Rate(1) # 1hz
        # Night gathers, and now my watch begins. It shall not end until my death.
        # I shall take no wife, hold no lands, father no children.
        # I shall wear no crowns and win no glory.
        # I shall live and die at my post.
        # I am the sword in the darkness.
        # I am the watcher on the walls.
        # I am the fire that burns against cold, the light that brings the dawn, the horn that wakes the sleepers, the shield that guards the realms of men.
        # I pledge my life and honor to the Night's Watch, for this night and all the nights to come
        while not rospy.is_shutdown():
            rate.sleep()  # quick sleep for safety
            self.update()

    def update(self):
        """
        Update function to call from a looping thread.
        """
        if _ROCON:

            try:
                new_conns, lost_conns = self.connections.update()

                if len(new_conns[PUBLISHER]) > 0 or len(lost_conns[PUBLISHER]) > 0:
                    self.topics_change_cb([ c.name for c in new_conns[PUBLISHER]], [c.name for c in lost_conns[PUBLISHER]])
                if len(new_conns[SUBSCRIBER]) > 0 or len(lost_conns[SUBSCRIBER]) > 0:
                    self.topics_change_cb([ c.name for c in new_conns[SUBSCRIBER]], [c.name for c in lost_conns[SUBSCRIBER]])

                if len(new_conns[SERVICE]) > 0 or len(lost_conns[SERVICE]) > 0:
                    self.services_change_cb([c.name for c in new_conns[SERVICE]], [c.name for c in lost_conns[SERVICE]])

                if len(new_conns[ACTION_SERVER]) > 0 or len(lost_conns[ACTION_SERVER]) > 0:
                    #FIXME : do we need clients here ??
                    self.actions_change_cb([c.name for c in new_conns[ACTION_SERVER]], [c.name for c in lost_conns[ACTION_SERVER]])

            except rospy.ROSException:
                rospy.logerr("ROS Watcher : Connections list unavailable.")
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Watcher : ros shutdown while looking for Connections .")

        else:
            try:
                publishers, subscribers, services = self._master.getSystemState()

                new_publishers = [pub[0] for pub in publishers if pub not in self.publishers]
                lost_publishers = [pub[0] for pub in self.publishers if pub not in publishers]

                new_services = [srv[0] for srv in services if srv not in self.services]
                lost_services = [srv[0] for srv in self.services if srv not in services]

                if len(new_publishers) > 0 or len(lost_publishers) > 0:
                    self.topics_change_cb(new_publishers, lost_publishers)

                if len(new_services) > 0 or len(lost_services) > 0:
                    self.services_change_cb(new_services, lost_services)

                # TODO : find a simple way to detect actions ( or drop support for it if it s too much useless code ? )

                self.publishers = publishers
                self.subscribers = subscribers
                self.services = services

            except socket.error:
                rospy.logerr("Gateway : couldn't get system state from the master "
                             "[did you set your master uri to a wireless IP that just went down?]")
