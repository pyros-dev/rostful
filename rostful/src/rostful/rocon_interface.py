# -*- coding: utf-8 -*-
from __future__ import absolute_import


import threading

import roslib
import rospy

import rocon_interactions
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions

from rocon_interactions.rapp_watcher import RappWatcher
from .interaction_watcher import InteractionWatcher

import rocon_app_manager_msgs.msg as rocon_app_manager_msgs
import rocon_app_manager_msgs.srv as rocon_app_manager_srvs

from rocon_app_manager_msgs.msg import Status, RappList
from rocon_app_manager_msgs.srv import StartRapp, StopRapp

from rosinterface import exceptions
from rosinterface.exceptions import(
                        FailedToStartRappError,
                        FailedToStopRappError,
                        FailedToListRappsError,
                        )

import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri
import ast

"""
Interface with ROCON.
Gather rocon information and store them ( as dict to be serialized to json easily by the rostful server ) TODO
No inheritance to make sure destructor is called properly.
"""
class RoconInterface():

    def __init__(self):
        #current rapps namespaces and interactions exposed
        self.rapps_namespaces = {}
        self.running_rapps_namespaces = {}
        self.interactions = {}
        #current rapps namespaces and interactions we are waiting for
        self.rapps_namespaces_waiting = []
        self.interactions_waiting = []
        #last requested rapps namespaces and interactions to be exposed
        self.rapps_namespaces_args = []
        self.interactions_args = []

        rapps_ns_args = ast.literal_eval(rospy.get_param('~rapps_namespaces', "[]"))
        interactions_args = ast.literal_eval(rospy.get_param('~interactions', "[]"))

        self.rapp_watcher = RappWatcher( self._namespaces_change_cb, self._rapp_status_changed )
        self.rapp_watcher.start()

        self.expose_rapps(rapps_ns_args)

        self.interaction_watcher = InteractionWatcher(self._interaction_status_changed)
        self.interaction_watcher.start()

        self.expose_interactions(interactions_args)

    def reconfigure(self, config, level):

        rospy.logwarn("""Reconfigure Request: \rapps_namespaces : {rapps_namespaces}""".format(**config))
        new_rapps = ast.literal_eval(config["rapps_namespaces"])
        self.expose_rapps(new_rapps)

        #LATER
        rospy.logwarn("""Reconfigure Request: \ninteractions : {interactions}""".format(**config))
        new_interactions = ast.literal_eval(config["interactions"])
        self.expose_interactions(new_interactions)

        return config

    def _namespaces_change_cb(self, added_namespaces, removed_namespaces):
        ns_to_watch=[]
        for ns, ns_ori in [ (n.strip("/"),n) for n in added_namespaces ]: # careful with enclosing /
            if ns in self.rapps_namespaces_waiting:
                self.rapps_namespaces[ns] = {} #preparing dictionary to hold Rapps Info
                self.running_rapps_namespaces[ns] = {} #preparing dictionary to hold Rapps Info
                self.rapps_namespaces_waiting.remove(ns)
                ns_to_watch.append(ns_ori)
        for ns in [ n.strip("/") for n in removed_namespaces ]:
            if ns in self.running_rapps_namespaces.keys():
                del self.running_rapps_namespaces[ns]
            if ns in self.rapps_namespaces.keys():
                del self.rapps_namespaces[ns]
                self.rapps_namespaces_waiting.append(ns)

        return ns_to_watch

    def _rapp_status_changed(self, namespace, added_available_rapps, removed_available_rapps, added_running_rapps, removed_running_rapps):
        namespace = namespace.strip("/") # remove potential parasits characters #TODO : check absolute/relative naming
        if namespace in self.rapps_namespaces.keys() :
            for k, v in added_available_rapps.iteritems():
                self.rapps_namespaces[namespace][k] = v
                rospy.loginfo('found rapp in %r : %r', namespace, k)

            for k, v in removed_available_rapps.iteritems():
                if k in self.rapps_namespaces[namespace].keys():
                    del self.rapps_namespaces[namespace][k]
                    rospy.loginfo('removed rapp in %r : %r', namespace, k)

            for k, v in added_running_rapps.iteritems():
                self.running_rapps_namespaces[namespace][k] = v
                rospy.loginfo('started rapp in %r : %r', namespace, k)

            for k, v in removed_running_rapps.iteritems():
                if k in self.running_rapps_namespaces[namespace].keys():
                    del self.running_rapps_namespaces[namespace][k]
                    rospy.loginfo('stopped rapp in %r : %r', namespace, k)

    def _interaction_status_changed(self, added_interactions, removed_interactions):
        for i in added_interactions:
            self.interactions[i.name] = i
            rospy.loginfo('found interaction %r', i.name)

        for i in removed_interactions:
            if i.name in self.interactions.keys():
                del self.interactions[i.name]
                rospy.loginfo('removed interaction %r', i.name)

    def add_rapp_ns(self, rapp_ns):
        rapp_ns = rapp_ns.strip("/") # normalizing ns names #TODO : check absolute/relative naming
        if not rapp_ns in self.rapps_namespaces :
            self.rapps_namespaces_waiting.append(rapp_ns)
        return True

    def del_rapp_ns(self, rapp_ns):
        rapp_ns = rapp_ns.strip("/") # normalizing ns names #TODO : check absolute/relative naming
        if rapp_ns in self.rapps_namespaces.keys():
            del self.rapps_namespaces[rapp_ns]
        elif rapp_ns in self.rapps_namespaces_waiting:
            self.rapps_namespaces_waiting.remove(rapp_ns)
        return True

    """
    This exposes a list of rapps.
    """
    def expose_rapps(self, rapp_namespaces):
        rospy.logwarn('Exposing Rapps Namespaces : %r', rapp_namespaces)
        if not rapp_namespaces:
            return
        for rapp_ns in rapp_namespaces:
            if not rapp_ns in self.rapps_namespaces_args:
                ret = self.add_rapp_ns(rapp_ns)
                if ret: rospy.loginfo( 'Added Rapp Namespace %s', rapp_ns )

        for rapp_ns in self.rapps_namespaces_args:
            if not rapp_ns in rapp_namespaces:
                ret = self.del_rapp_ns(rapp_ns)
                if ret: rospy.loginfo ( 'Removed Rapp Namespace %s', rapp_ns )

        #Updating the list of Rapps Namespaces
        self.rapps_namespaces_args = [ n.strip("/") for n in rapp_namespaces ]  # normalizing ns names #TODO : check absolute/relative naming


    def expose_interactions(self, args):
        pass
