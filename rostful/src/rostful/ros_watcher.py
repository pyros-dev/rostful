# -*- coding: utf-8 -*-
from __future__ import absolute_import
from concurrent import futures
import threading

import roslib
import rospy

import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs

from rocon_app_manager_msgs.msg import Status, RappList
from rocon_app_manager_msgs.srv import StartRapp, StopRapp


from .interaction import Interaction
from .interaction_table import InteractionsTable

import rocon_interactions
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions


import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri


from rocon_python_comms import connections, PUBLISHER, SUBSCRIBER, SERVICE, ACTION_SERVER, ACTION_CLIENT

class ROSWatcher(threading.Thread):  #TODO : DO NOT inherit from thread. instead use the executor for watching.

    def __init__(self, topics_change_cb, services_change_cb, actions_change_cb):  # TODO : use Queue for callbacks to be executed in main thread.
        super(ROSWatcher, self).__init__()

        self.executor = futures.ThreadPoolExecutor(max_workers=1)

        #TODO : acessing members hould be done via property that copy the object to avoid accidental modification during use

        #current services topics and actions available
        self.connections = connections.ConnectionCache()
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
        try:
            new_conns, lost_conns = self.connections.update()

            if len(new_conns[PUBLISHER]) > 0 or len(lost_conns[PUBLISHER]) > 0:
                self.topics_change_cb(new_conns[PUBLISHER], lost_conns[PUBLISHER])

            if len(new_conns[SERVICE]) > 0 or len(lost_conns[SERVICE]) > 0:
                self.services_change_cb(new_conns[SERVICE], lost_conns[SERVICE])

            if len(new_conns[ACTION_SERVER]) > 0 or len(lost_conns[SERVICE]) > 0:
                #FIXME : do we need clients here ??
                self.actions_change_cb(new_conns[ACTION_SERVER], lost_conns[ACTION_SERVER])

        except rospy.ROSException:
            rospy.logerr("ROS Watcher : Connections list unavailable.")
        except rospy.ROSInterruptException:
            rospy.logerr("ROS Watcher : ros shutdown while looking for Connections .")

