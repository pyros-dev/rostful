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

class InteractionWatcher(threading.Thread):  #TODO : DO NOT inherit from thread. instead use the executor for watching.

    def __init__(self, interactions_change_cb):  # TODO : use Queue for callbacks to be executed in main thread.
        """
        @param namespaces_change_cb : callback for a change of namespaces
        @param ns_status_change_cb : callback for a change of namespace status
        @param rapp_status_change_cb : callback for a change of rapp status
        """
        super(InteractionWatcher, self).__init__()

        self.executor = futures.ThreadPoolExecutor(max_workers=1)

        #TODO : acessing members hould be done via property that copy the object to avoid accidnetal modification during use
        self.interactions_table = InteractionsTable()
        self.interactions = {}
        self.interactions_change_cb = interactions_change_cb

        self.roles = []

        self.get_interactions_service_proxy = None
        self.get_roles_service_proxy = None
        self.request_interaction_service_proxy = None

        self.rocon_uri = rocon_uri.parse(
            "rocon:/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "/" + rocon_std_msgs.Strings.URI_WILDCARD
            + "|" + rocon_std_msgs.Strings.OS_CHROME
        )
        self.platform_info = rocon_std_msgs.PlatformInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                         uri=str(self.rocon_uri),
                                                         icon=rocon_std_msgs.Icon()
                                                         )

    def _hook_interaction_manager(self, interactions_namespace):
        """
            setting up interaction manager apis. and check if the services are available
            :param: interactions_namespace
            :returns: service and pubs apis
            :rtype: dict
        """
        iamgr_services = {}
        iamgr_services['get_interactions'] = interactions_namespace + '/' + 'get_interactions'
        iamgr_services['get_roles'] = interactions_namespace + '/' + 'get_roles'
        iamgr_services['request_interaction'] = interactions_namespace + '/' + 'request_interaction'
        iamgr_pubs = {}
        iamgr_pubs['interactions_update'] = interactions_namespace + '/' + 'interactions_update'

        for service_name in iamgr_services.keys():
            if not rocon_python_comms.service_is_available(iamgr_services[service_name]):
                raise rocon_python_comms.NotFoundException("'%s' service is not validated" % service_name)

        if not self.get_interactions_service_proxy :
            self.get_interactions_service_proxy = rospy.ServiceProxy(iamgr_services['get_interactions'], rocon_interaction_srvs.GetInteractions)
        if not self.get_roles_service_proxy :
            self.get_roles_service_proxy = rospy.ServiceProxy(iamgr_services['get_roles'], rocon_interaction_srvs.GetRoles)
        if not self.request_interaction_service_proxy :
            self.request_interaction_service_proxy = rospy.ServiceProxy(iamgr_services['request_interaction'], rocon_interaction_srvs.RequestInteraction)

        rospy.Subscriber(iamgr_pubs['interactions_update'], rocon_interaction_msgs.InteractionsUpdate, self._ia_update)

        return iamgr_services, iamgr_pubs

    def _ia_update(self, msg):
        if msg.update:
            # we need to refresh our interaction list
            self.get_interactions()

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
        interactions_namespace = ''
        while not rospy.is_shutdown():
            rate.sleep()  # quick sleep for safety
            # init loop
            while interactions_namespace == '':  # we enter here only once, for initialization
                rate.sleep()  # quick sleep for safety

                try:
                    rospy.logdebug("InteractionWatcher : Get interactions service Handles")
                    interactions_namespace = rocon_python_comms.find_service_namespace('get_interactions', 'rocon_interaction_msgs/GetInteractions', unique=True)
                    self._hook_interaction_manager(interactions_namespace)

                except rospy.ROSException:
                    rospy.logerr("Interactions : interaction manager services unavailable.")
                except rospy.ROSInterruptException:
                    rospy.logerr("Interactions : ros shutdown while looking for the rapp manager services.")

                # initializing interactions after we found the services.
                self.get_interactions()

            pass  # nothing to do here at the moment

    def get_interactions(self):
        if self.get_interactions_service_proxy :
            call_result = self.get_interactions_service_proxy([], self.platform_info.uri)
            added = []
            removed = []
            for msg in call_result.interactions:
                interaction = Interaction(msg)
                if not self.interactions_table.exists(interaction):
                    self.interactions_table.append(interaction)
                    added.append(interaction)

            for i in self.interactions_table:
                if not i.hash in [ i.hash for i in call_result.interactions ]:
                    self.interactions_table.remove(interaction)
                    removed.append(interaction)

            self.interactions_change_cb(added, removed)
            rospy.loginfo('Loaded %r Interactions', len(self.interactions_table))
            self.interactions = {i.name: i for i in self.interactions_table}
        return self.interactions

    def get_roles(self):
        if self.get_roles_service_proxy :
            call_result = self.get_roles_service_proxy(self.platform_info.uri)
            added = []
            removed = []
            for role in call_result.roles:
                if not role in self.roles :
                    self.roles.append(role)
                    added.append(role)
            for r in list(self.roles):  # duplicating list to allow modification of original
                if not r in call_result.roles:
                    self.roles.remove(role)
                    removed.append(role)
        return self.roles

    def request_interaction(self, *args, **kvargs):
        """
        Synchronous call on request interaction
        :param args:
        :param kvargs:
        :return:
        """
        if self.request_interaction_service_proxy :
            call_result = self.request_interaction_service_proxy(rocon_interaction_srvs.RequestInteractionRequest(*args, **kvargs))
            rospy.logwarn('Request_InterAction response : %r', call_result)
            return call_result
        return None

    def request_interaction_async(self, *args, **kvargs):
        """
        Asynchronous call on request interaction
        :param args:
        :param kvargs:
        :return: future
        """
        return self.executor.submit(self.request_interaction, *args, **kvargs)

    def __del__(self):
        if self.executor:
            self.executor.shutdown()

# class InteractionWatcher(threading.Thread)
