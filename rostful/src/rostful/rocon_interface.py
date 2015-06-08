# -*- coding: utf-8 -*-
from __future__ import absolute_import


import threading

import roslib
import rospy

_ROCON = False
try:
    from rocon_interactions.rapp_watcher import RappWatcher
    from .interaction_watcher import InteractionWatcher

    import rocon_app_manager_msgs.msg as rocon_app_manager_msgs

    _ROCON = True
except Exception, e:
    rospy.logwarn('Missing rocon codebase. Rocon features disabled')


import ast

"""
Interface with ROCON.
Gather rocon information and store them ( as dict to be serialized to json easily by the rostful server ) TODO
No inheritance to make sure destructor is called properly.
"""
class RoconInterface(object):

    def __init__(self, ros_interface):
        #current rapps namespaces and interactions exposed
        self.rapps_namespaces = {}
        self.running_rapp_namespaces = {}
        self.interactions = {}
        #current rapps namespaces and interactions we are waiting for
        self.rapps_namespaces_waiting = []
        self.interactions_waiting = []
        #last requested rapps namespaces and interactions to be exposed
        self.rapps_namespaces_args = []
        self.interactions_args = []

        # We need to use of ROS interface to expose Topics/services/actions when needed.
        self.ros_interface = ros_interface

        rapps_ns_args = ast.literal_eval(rospy.get_param('~rapps_namespaces', "[]"))
        interactions_args = ast.literal_eval(rospy.get_param('~interactions', "[]"))

        if _ROCON:
            #TODO : Rapp Watcher shouldnt even get started when we are running on concert. It s useful only on Robot.
            self.rapp_watcher = RappWatcher( self._namespaces_change_cb, self._available_rapps_list_changed, self._running_rapp_status_changed, silent_timeout=True)
            self.rapp_watcher.start()

            self.expose_rapps(rapps_ns_args)

            self.interaction_watcher = InteractionWatcher(self._interaction_status_changed)
            self.interaction_watcher.start()

            self.expose_interactions(interactions_args)
        else:
            pass  # we dont want to do anything without rocon codebase

    def reconfigure(self, config, level):

        rospy.logwarn("""Reconfigure Request: \rapps_namespaces : {rapps_namespaces}""".format(**config))
        if _ROCON:
            new_rapps = ast.literal_eval(config["rapps_namespaces"])
            self.expose_rapps(new_rapps)
        else:
            pass

        #LATER
        rospy.logwarn("""Reconfigure Request: \ninteractions : {interactions}""".format(**config))
        if _ROCON:
            new_interactions = ast.literal_eval(config["interactions"])
            self.expose_interactions(new_interactions)
        else:
            pass

        return config

#### ROCON ONLY BEGIN
    if _ROCON:
        def _namespaces_change_cb(self, added_namespaces, removed_namespaces):
            ns_to_watch=[]
            for ns, ns_ori in [ (n.strip("/"),n) for n in added_namespaces ]: # careful with enclosing /
                if ns in self.rapps_namespaces_waiting:
                    self.rapps_namespaces[ns] = {} #preparing dictionary to hold Rapps Info
                    self.running_rapp_namespaces[ns] = {} #preparing dictionary to hold Rapps Info
                    self.rapps_namespaces_waiting.remove(ns)
                    ns_to_watch.append(ns_ori)
            for ns in [ n.strip("/") for n in removed_namespaces ]:
                if ns in self.running_rapp_namespaces.keys():
                    del self.running_rapp_namespaces[ns]
                if ns in self.rapps_namespaces.keys():
                    del self.rapps_namespaces[ns]
                    self.rapps_namespaces_waiting.append(ns)

            return ns_to_watch

        def _available_rapps_list_changed(self, namespace, added_available_rapps, removed_available_rapps):
            namespace = namespace.strip("/") # remove potential parasits characters #TODO : check absolute/relative naming
            if namespace in self.rapps_namespaces.keys() :
                for k, v in added_available_rapps.iteritems():
                    self.rapps_namespaces[namespace][k] = v
                    rospy.loginfo('found rapp in %r : %r', namespace, k)

                for k, v in removed_available_rapps.iteritems():
                    if k in self.rapps_namespaces[namespace].keys():
                        del self.rapps_namespaces[namespace][k]
                        rospy.loginfo('removed rapp in %r : %r', namespace, k)

        def _running_rapp_status_changed(self, namespace, rapp_status, rapp):
            namespace = namespace.strip("/")  # remove potential parasites characters #TODO : check absolute/relative naming
            if namespace in self.rapps_namespaces.keys():
                if rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING:
                    self.running_rapp_namespaces[namespace] = rapp
                    rospy.loginfo('started rapp in %r : %r', namespace, rapp['display_name'])
                elif rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED and namespace in self.running_rapp_namespaces:
                    del self.running_rapp_namespaces[namespace]
                    if 'display_name' in rapp.keys():
                        rospy.loginfo('stopped rapp in %r : %r', namespace, rapp['display_name'])

        def _interaction_status_changed(self, added_interactions, removed_interactions):
            for i in added_interactions:
                self.interactions[i.name] = i
                rospy.loginfo('found interaction %r', i.name)

            for i in removed_interactions:
                if i.name in self.interactions.keys():
                    del self.interactions[i.name]
                    rospy.loginfo('removed interaction %r', i.name)
#### ROCON ONLY END

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
                if ret:
                    rospy.loginfo('Added Rapp Namespace %s', rapp_ns)

        for rapp_ns in self.rapps_namespaces_args:
            if not rapp_ns in rapp_namespaces:
                ret = self.del_rapp_ns(rapp_ns)
                if ret:
                    rospy.loginfo('Removed Rapp Namespace %s', rapp_ns)

        #Updating the list of Rapps Namespaces
        self.rapps_namespaces_args = [ n.strip("/") for n in rapp_namespaces ]  # normalizing ns names #TODO : check absolute/relative naming

    def get_interactions(self):
        return self.interactions

    def request_interaction(self, name):
        if name in self.interactions.keys():
            # FIXME : pass actual remocon id.
            req_res = self.interaction_watcher.request_interaction(remocon='rostful', hash=self.interactions[name].hash)

            if not req_res.result:
                rospy.logerr('ERROR %r requesting interaction : %r', req_res.error_code, req_res.message)
            else:
                if self.interactions[name].required and self.interactions[name].required.rapp:
                    # exposing public interface via ROS interface
                    for ns, rapp in self.running_rapp_namespaces.iteritems():
                        if rapp['name'] == self.interactions[name].required.rapp:
                            rospy.loginfo('exposing interface : %r', rapp['public_interface'])
                            for ielem in rapp['public_interface']:
                                if ielem.key == 'services':
                                    for e in ast.literal_eval(ielem.value):
                                        rospy.loginfo('exposing service %r', e)
                                        if 'name' in e.keys():
                                            self.ros_interface.add_service(e['name'])
                                elif ielem.key == 'publishers' or ielem.key == 'subscribers':
                                    for e in ast.literal_eval(ielem.value):
                                        rospy.loginfo('exposing topic %r', e)
                                        if 'name' in e.keys():
                                            self.ros_interface.add_topic(e['name'])
                                elif ielem.key == 'actions_servers' or ielem.key == 'actions_clients':
                                    for e in ast.literal_eval(ielem.value):
                                        rospy.loginfo('exposing action server %r', e)
                                        if 'name' in e.keys():
                                            self.ros_interface.add_action(e['name'])
            return req_res

        else:
            return False  # FIXME : find proper error return code here.

    def expose_interactions(self, args):
        pass
