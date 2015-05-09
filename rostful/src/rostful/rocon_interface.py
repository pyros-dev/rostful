# -*- coding: utf-8 -*-
from __future__ import absolute_import


import threading

import roslib
import rospy

import rocon_interactions
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions

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


def list_rapp_msg_to_dict(list_rapp):
    """
    convert msg to dict
    """
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = {}
        dict_rapp[name]["status"] = rapp.status
        dict_rapp[name]["name"] = rapp.name
        dict_rapp[name]["display_name"] = rapp.display_name
        dict_rapp[name]["description"] = rapp.description
        dict_rapp[name]["compatibility"] = rapp.compatibility
        dict_rapp[name]["preferred"] = rapp.preferred
        dict_rapp[name]["icon"] = rapp.icon
        dict_rapp[name]["implementations"] = rapp.implementations
        dict_rapp[name]["public_interface"] = rapp.public_interface
        dict_rapp[name]["public_parameters"] = rapp.public_parameters
    return dict_rapp


#TODO : find more standard/optimized/"we dont have to maintain it" way of doing this ?
def _dict_key_diff(current_dict, past_dict):
    set_current, set_past = set(current_dict.keys()), set(past_dict.keys())
    intersect = set_current.intersection(set_past)
    added = set_current - intersect
    removed = set_past - intersect
    changed = set(o for o in intersect if past_dict[o] != current_dict[o])
    unchanged = set(o for o in intersect if past_dict[o] == current_dict[o])

    return (dict((el,current_dict[el]) for el in added),
            dict((el,past_dict[el]) for el in removed),
            dict((el,current_dict[el]) for el in changed),
            dict((el,current_dict[el]) for el in unchanged))

"""
Interface with ROCON.
No inheritance to make sure destructor is called properly.
"""
class RoconInterface():

    class RappWatcher(threading.Thread):

        class WatchedNS():
            def __init__(self, name, rapp_status_change_cb):

                self.name = name
                self.rapp_status_change_cb = rapp_status_change_cb

                self.start_rapp = None
                self.stop_rapp = None
                self.list_rapps = None
                self.status_subscriber = None
                self.rapplist_subscriber = None

                self._available_rapps = {}
                self._running_rapps = {}

            def grab_start_rapp(self, start_rapp_srvs_list):
                if not self.start_rapp :
                    for s in start_rapp_srvs_list :
                        if s.startswith(self.name): # if we have found the service name that match our namespace
                            self.start_rapp = rospy.ServiceProxy(s, rocon_app_manager_srvs.StartRapp)
                            return True
                    return False # we couldnt grab anything
                return True # we re good, no need for it

            def grab_stop_rapp(self, stop_rapp_srvs_list):
                if not self.stop_rapp :
                    for s in stop_rapp_srvs_list :
                        if s.startswith(self.name): # if we have found the service name that match our namespace
                            self.stop_rapp = rospy.ServiceProxy(s, rocon_app_manager_srvs.StopRapp)
                            return True
                    return False # we couldnt grab anything
                return True # we re good, no need for it

            def grab_list_rapps(self, list_rapps_srvs_list):
                if not self.list_rapps :
                    for s in list_rapps_srvs_list :
                        if s.startswith(self.name): # if we have found the service name that match the namespace
                            self.list_rapps = rospy.ServiceProxy(s, rocon_app_manager_srvs.GetRappList)
                            return True
                    return False # we couldnt grab anything
                return True # we re good, no need for it

            def grab_status_subscriber(self, status_topics_list):
                if not self.status_subscriber :
                    for t in status_topics_list :
                        if t.startswith(self.name): # if we have found the service name that match the namespace
                            self.status_subscriber = rospy.Subscriber(t, rocon_app_manager_msgs.Status, self._ros_status_subscriber)
                            return True
                    return False # we couldnt grab anything
                return True # we re good, no need for it

            def grab_rapplist_subscriber(self, rapplist_topics_list):
                if not self.rapplist_subscriber :
                    for t in rapplist_topics_list :
                        if t.startswith(self.name): # if we have found the service name that match the namespace
                            self.rapplist_subscriber = rospy.Subscriber(t, rocon_app_manager_msgs.RappList, self._process_rapp_list_msg)
                            #Latched Topic : First time we connect, we should get the list of rapps
                            return True
                    return False # we couldnt grab anything
                return True # we re good, no need for it

            def _process_rapp_list_msg(self, msg):
                """
                Update the available rapp list

                @param data: information of rapps
                @type rocon_app_manager_msgs/RappList
                """

                added_available_rapps, removed_available_rapps, _, _ = _dict_key_diff(list_rapp_msg_to_dict(msg.available_rapps),self._available_rapps)
                added_running_rapps, removed_running_rapps, _, _ = _dict_key_diff(list_rapp_msg_to_dict(msg.running_rapps),self._running_rapps)

                self.rapp_status_change_cb( self.name, added_available_rapps, removed_available_rapps, added_running_rapps, removed_running_rapps)

                rospy.logwarn('updating available rapps list : %r', [r.name for r in msg.available_rapps])
                self._available_rapps = list_rapp_msg_to_dict(msg.available_rapps)
                rospy.logwarn('updating running rapps list : %r', [r.name for r in msg.running_rapps])
                self._running_rapps = list_rapp_msg_to_dict(msg.running_rapps)

            def _ros_status_subscriber(self, msg):
                """
                """

                #rospy.logwarn('NEW RAPP STATUS DATA : %r',msg)

                #old_running_status = self.is_running
                #self.is_running = (msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_RUNNING)
                #if old_running_status and msg.rapp_status == rocon_app_manager_msgs.Status.RAPP_STOPPED:
                #    self.status_callback()  # let the higher level disable pairing mode via this

            @property
            def available_rapps_list(self):#TODO : remove '_list' from name. misleading
                return self._available_rapps

            @property
            def running_rapps_list(self):#TODO : remove '_list' from name. misleading
                return self._running_rapps

            #####class WatchedNS


        def __init__(self, namespaces_change_cb, rapp_status_change_cb, get_rapp_list_service_name = 'list_rapps'):
            """
            @param interaction_status_changed_cb : callback for a change of interaction status
            @param rapp_status_change_cb : callback for a change of rapp status
            """
            super(RoconInterface.RappWatcher, self).__init__()
            #servicename ( to be able to extract namespace )
            self.get_rapp_list_service_name = get_rapp_list_service_name
            #contains data by namespace
            self.watching_ns={}
            # all available namespace detected
            self._available_namespaces = []
            # all namespace that are completely connected
            self._watched_namespaces = []

            #TODO : check callback signature
            self.rapp_status_change_cb = rapp_status_change_cb
            self.namespaces_change_cb = namespaces_change_cb

        def run(self):
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
                rate.sleep() # quick sleep for safety
                try:
                    # long timeout because no point of keep going if we cannot find this one.
                    list_rapps_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/GetRappList', timeout=rospy.rostime.Duration(60.0), unique=False)

                    # Detecting new namespaces
                    ns_added = []
                    ns_removed = []

                    for ns in self._available_namespaces :
                        found = False
                        for fname in list_rapps_service_names :
                            if fname.startswith(ns) :
                                found = True
                        if not found :
                            ns_removed.append(ns)
                            #stopping to watch removed namespaces
                            if ns in self.watching_ns.keys() :
                                del self.watching_ns[ns]
                                rospy.logerr('STOPPED WATCHING : %r ', ns)

                    for fname in list_rapps_service_names :
                        if fname.endswith(self.get_rapp_list_service_name):
                            ns = fname[:-len(self.get_rapp_list_service_name)]
                            if ns not in self._available_namespaces :
                                self._available_namespaces.append(ns)
                                ns_added.append(ns)

                    if 0 < len(ns_added) or 0 < len(ns_removed) :
                        to_watch = self.namespaces_change_cb(ns_added, ns_removed)
                        for ns in to_watch :
                            if ns in self._available_namespaces :
                                rospy.logerr('NOW WATCHING FOR RAPPS IN %r ', ns)
                                self.watching_ns[ns] = self.WatchedNS(ns, self.rapp_status_change_cb)

                    #grabing all services & topics
                    if set(self.watching_ns.keys()) != set(self._watched_namespaces): # if some namespaces are not fully connected
                        start_rapp_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/StartRapp', timeout=rospy.rostime.Duration(1.0), unique=False)
                        stop_rapp_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/StopRapp', timeout=rospy.rostime.Duration(1.0), unique=False)
                        status_topic_names = rocon_python_comms.find_topic('rocon_app_manager_msgs/Status', timeout=rospy.rostime.Duration(1.0), unique=False)
                        rapplist_topic_names = rocon_python_comms.find_topic('rocon_app_manager_msgs/RappList', timeout=rospy.rostime.Duration(1.0), unique=False)

                        for ns, ns_data in self.watching_ns.iteritems() :
                            if ns not in self._watched_namespaces : # if that namespace is not fully connected
                                connect = ns_data.grab_start_rapp(start_rapp_service_names)
                                connect = ns_data.grab_stop_rapp(stop_rapp_service_names) and connect
                                connect = ns_data.grab_list_rapps(list_rapps_service_names) and connect
                                connect = ns_data.grab_rapplist_subscriber(rapplist_topic_names) and connect
                                connect = ns_data.grab_status_subscriber(status_topic_names) and connect
                                if connect :
                                    #if all services and topics are connected this namespace is considered connected
                                    self._watched_namespaces.append(ns)


                    #TODO : survive if services/topics ever go down... and be able to catch them again when they come back.

                except rospy.ROSException:
                    rospy.logerr("Interactions : rapp manager services disappeared.")
                except rospy.ROSInterruptException:
                    rospy.logerr("Interactions : ros shutdown while looking for the rapp manager services.")

        def get_available_rapps(self, namespace):
            self.watching_ns[namespace].available_rapps_list

        def get_running_rapps(self, namespace):
            self.watching_ns[namespace].running_rapps_list

        #####class RappWatcher(threading.Thread)

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

        self.rapp_watcher = self.RappWatcher( self._namespaces_change_cb, self._rapp_status_changed )
        self.rapp_watcher.start()

        self.expose_rapps(rapps_ns_args)
        #LATER
        #self.expose_interactions(interactions_args)

    def reconfigure(self, config, level):

        rospy.logwarn("""Reconfigure Request: \rapps_namespaces : {rapps_namespaces}""".format(**config))
        new_rapps = ast.literal_eval(config["rapps_namespaces"])
        self.expose_rapps(new_rapps)

        #LATER
        #rospy.logwarn("""Reconfigure Request: \ninteractions : {interactions}""".format(**config))
        #new_interactions = ast.literal_eval(config["interactions"])
        #self.expose_interactions(new_interactions)

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

    def _interaction_status_changed(self):
        pass

    def add_rapp_ns(self, rapp_ns):
        rapp_ns = rapp_ns.strip("/") # normalizing ns names #TODO : check absolute/relative naming
        if not rapp_ns in self.rapps_namespaces :
            self.rapps_namespaces_waiting.append(rapp_ns)
        return True

    def del_rapp_ns(self, rapp_ns):
        rapp_ns = rapp_ns.strip("/") # normalizing ns names #TODO : check absolute/relative naming
        if rapp_ns in self.rapps_namespaces.keys():
            del self.rapps_namespaces[rapp_ns]
        elif rapp_ns in rapps_namespaces_waiting :
            sel.rapps_namespaces_waiting.remove(rapp_ns)
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
