#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of turtles across multimaster
# boundaries. Typically turtlesim clients would connect to the kill and
# spawn services directly to instantiate themselves, but since we can't
# flip service proxies, this is not possible. So this node is the inbetween
# go-to node and uses a rocon service pair instead.
#
# It supplements this relay role with a bit of herd management - sets up
# random start locations and feeds back aliased names when running with
# a concert.

##############################################################################
# Imports
##############################################################################

import copy
import math
import os
import random
import tempfile

import concert_service_utilities
import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_launch
import rospy
import rocon_gateway_utils
import rocon_python_comms
import std_msgs.msg as std_msgs
import turtlesim.srv as turtlesim_srvs

##############################################################################
# Utilities
##############################################################################


def prepare_launch_configurations(turtles):
    """
    :param Turtle[] turtles:
    """
    port = 11
    launch_text = '<concert>\n'
    for turtle in turtles:
        launch_text += '  <launch title="%s:114%s" package="rostful_examples" name="turtle.launch" port="114%s">\n' % (turtle.unique_name, str(port), str(port))
        launch_text += '    <arg name="turtle_name" value="%s"/>\n' % turtle.unique_name
        launch_text += '    <arg name="turtle_concert_whitelist" value="%s"/>\n' % str(turtle.concert_whitelist)  # e.g. [Turtle Concert, Turtle Teleop Concert, Concert Tutorial]
        launch_text += '    <arg name="turtle_rapp_whitelist" value="%s"/>\n' % str(turtle.rapp_whitelist)  # e.g. [rocon_apps, turtle_concert]
        launch_text += '    <arg name="disable_zeroconf" value="%s"/>\n' % str(turtle.disable_zeroconf)  # e.g. [True, False]
        launch_text += '    <arg name="rostful_port" value="%s"/>\n' % str(turtle.rostful_port)  # e.g. [True, False]
        launch_text += '    <arg name="rosbridge_port" value="%s"/>\n' % str(turtle.rosbridge_port)  # e.g. [True, False]
        launch_text += '  </launch>\n'
        port = port + 1
    launch_text += '</concert>\n'
    temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
    #print("\n" + console.green + rocon_launch_text + console.reset)
    temp.write(launch_text)
    #rospy.logwarn("Turtle Herder: rocon launch text\n%s" % launch_text)
    temp.close()  # unlink it later
    launch_configurations = rocon_launch.parse_rocon_launcher(temp.name, "--screen")
    try:
        os.unlink(temp.name)
    except OSError:
        rospy.logerr("Turtle Herder : failed to unlink the rocon launcher.")
    return launch_configurations


##############################################################################
# Turtle
##############################################################################

class Turtle(object):
    """
    Holds parameterised information used for customising a turtle spawning.
    """
    __slots__ = [
                 'name',
                 'unique_name',
                 'rapp_whitelist',
                 'concert_whitelist',
                 'disable_zeroconf',
                 'rosbridge_port',
                 'rostful_port'
                ]

    def __init__(self, name, rapp_whitelist, concert_whitelist, disable_zeroconf, rosbridge_port, rostful_port):
        self.name = name
        self.unique_name = name  # this gets manipulated later
        self.rapp_whitelist = rapp_whitelist
        self.concert_whitelist = concert_whitelist
        self.disable_zeroconf = disable_zeroconf
        self.rosbridge_port = rosbridge_port
        self.rostful_port = rostful_port

##############################################################################
# Turtle Herder
##############################################################################


class TurtleHerder(object):
    '''
      Shepherds the turtles!

      @todo get alised names from the concert client list if the topic is available

      @todo watchdog for killing turtles that are no longer connected.
    '''
    __slots__ = [
        'turtles',              # list of turtle name strings
        '_kill_turtle_service_client',
        '_spawn_turtle_service_client',
        '_gateway_flip_service',
        '_processes',
        '_temporary_files',     # temporary files that have to be unlinked later
        'is_disabled',          # flag set when service manager tells it to shut down.
        '_terminal',            # terminal to use to spawn concert clients
        '_shutdown_subscriber'
    ]

    def __init__(self):
        self.turtles = []
        self._processes = []
        self._temporary_files = []
        self.is_disabled = False
        # herding backend
        rospy.wait_for_service('kill')  # could use timeouts here
        rospy.wait_for_service('spawn')
        self._kill_turtle_service_client = rospy.ServiceProxy('kill', turtlesim_srvs.Kill, persistent=True)
        self._spawn_turtle_service_client = rospy.ServiceProxy('spawn', turtlesim_srvs.Spawn, persistent=True)
        # kill the default turtle that turtlesim starts with
        try:
            unused_response = self._kill_turtle_service_client("turtle1")
        except rospy.ServiceException:
            rospy.logerr("Turtle Herder : failed to contact the internal kill turtle service")
        except rospy.ROSInterruptException:
            rospy.loginfo("Turtle Herder : shutdown while contacting the internal kill turtle service")
            return
        self._shutdown_subscriber = rospy.Subscriber('shutdown', std_msgs.Empty, self.shutdown)
        # gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)
        # set up a terminal type for spawning
        try:
            self._terminal = rocon_launch.create_terminal()
        except (rocon_launch.UnsupportedTerminal, rocon_python_comms.NotFoundException) as e:
            rospy.logwarn("Turtle Herder : cannot find a suitable terminal, falling back to spawning inside the current one [%s]" % str(e))
            self._terminal = rocon_launch.create_terminal(rocon_launch.terminals.active)

    def _spawn_simulated_turtles(self, turtles):
        """
        Very important to have checked that the turtle names are unique
        before calling this method.

        :param Turtle[] turtles:
        """
        for turtle in turtles:
            internal_service_request = turtlesim_srvs.SpawnRequest(
                                                random.uniform(3.5, 6.5),
                                                random.uniform(3.5, 6.5),
                                                random.uniform(0.0, 2.0 * math.pi),
                                                turtle.unique_name)
            try:
                unused_internal_service_response = self._spawn_turtle_service_client(internal_service_request)
                self.turtles.append(turtle)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("TurtleHerder : failed to contact the internal spawn turtle service")
                continue
            except rospy.ROSInterruptException:
                rospy.loginfo("TurtleHerder : shutdown while contacting the internal spawn turtle service")
                continue

    def _launch_turtle_clients(self, turtles):
        """
        :param Turtle[] turtles:
        """
        # spawn the turtle concert clients
        launch_configurations = prepare_launch_configurations(turtles)
        for launch_configuration in launch_configurations:
            rospy.loginfo("Turtle Herder : launching turtle concert client %s on port %s" %
                      (launch_configuration.name, launch_configuration.port))
            #print("%s" % launch_configuration)
            process, meta_roslauncher = self._terminal.spawn_roslaunch_window(launch_configuration)
            self._processes.append(process)
            self._temporary_files.append(meta_roslauncher)

    def spawn_turtles(self, turtles):
        """
        :param Turtle[] turtles:
        """
        uniquely_named_turtles = self._establish_unique_names(turtles)
        self._spawn_simulated_turtles(uniquely_named_turtles)
        self._launch_turtle_clients(uniquely_named_turtles)
        self._send_flip_rules(uniquely_named_turtles, cancel=False)

    def _establish_unique_names(self, turtles):
        """
        Make sure the turtle names don't clash with currently spawned turtles.
        If they do, postfix them with an incrementing counter.

        :param Turtle[] turtles: list of new turtle names to uniquify.
        :returns: uniquified names for the turtles.
        :rtype Turtle[]: updated turtles
        """
        for turtle in turtles:
            name_extension = ''
            count = 0
            while turtle.name + name_extension in [turtle.name for turtle in self.turtles]:
                name_extension = '_' + str(count)
                count = count + 1
            turtle.unique_name = turtle.name + name_extension
        return turtles

    def _send_flip_rules(self, turtles, cancel):
        """
        :param Turtle[] turtles:
        """
        for turtle in turtles:
            rules = []
            rule = gateway_msgs.Rule()
            rule.node = ''
            rule.type = gateway_msgs.ConnectionType.SUBSCRIBER
            # could resolve this better by looking up the service info
            rule.name = "/services/turtlesim/%s/cmd_vel" % turtle.unique_name
            rules.append(copy.deepcopy(rule))
            rule.type = gateway_msgs.ConnectionType.PUBLISHER
            rule.name = "/services/turtlesim/%s/pose" % turtle.unique_name
            rules.append(copy.deepcopy(rule))
            # send the request
            request = gateway_srvs.RemoteRequest()
            request.cancel = cancel
            remote_rule = gateway_msgs.RemoteRule()
            remote_rule.gateway = turtle.unique_name
            for rule in rules:
                remote_rule.rule = rule
                request.remotes.append(copy.deepcopy(remote_rule))
            try:
                self._gateway_flip_service(request)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("TurtleHerder : failed to send flip rules")
                return
            except rospy.ROSInterruptException:
                rospy.loginfo("TurtleHerder : shutdown while contacting the gateway flip service")
                return

    def _ros_service_manager_disable_callback(self, msg):
        self.is_disabled = True

    def shutdown(self, msg=None):
        """
          - Send unflip requests
          - Cleanup turtles on the turtlesim canvas.
          - Shutdown spawned terminals

        :todo: this should go in a service manager callable ros callback where we can
        call disable on this service and bring it down without having to SIGINT it.
        """
        # cleaning turtles is probably not really important since
        # we always shutdown turtlesim and turtle_herder together.
        # for name in self.turtles:
        #     try:
        #         unused_internal_service_response = self._kill_turtle_service_client(name)
        #     except rospy.ServiceException:  # communication failed
        #         break  # quietly fail
        #     except rospy.ROSInterruptException:
        #         break  # quietly fail

        self._terminal.shutdown_roslaunch_windows(processes=self._processes,
                                                  hold=False)
        for temporary_file in self._temporary_files:
            #print("Unlinking %s" % temporary_file.name)
            try:
                os.unlink(temporary_file.name)
            except OSError as e:
                rospy.logerr("Turtle Herder : failed to unlink temporary file [%s]" % str(e))

##############################################################################
# Launch point
##############################################################################

if __name__ == '__main__':

    rospy.init_node('turtle_herder')
    (service_name, unused_service_description, service_priority, unused_service_id) = concert_service_utilities.get_service_info()
    turtles = []
    turtle_parameters = rospy.get_param('/services/' + service_name + '/turtles', {})
    # should check that turtle_parameters is a dict here
    for name, parameters in turtle_parameters.items():
        try:
            turtles.append(Turtle(name, parameters['rapp_whitelist'], parameters['concert_whitelist'], parameters['disable_zeroconf'], parameters['rosbridge_port'], parameters['rostful_port']))
        except KeyError as e:
            rospy.logerr("TurtleHerder : not all turtle parameters found for %s (req'd even if empty)[%s]" % (name, str(e)))
    rospy.loginfo("TurtleHerder : spawning turtles: %s" % [turtle.name for turtle in turtles])

    turtle_herder = TurtleHerder()
    turtle_herder.spawn_turtles(turtles)
    while not rospy.is_shutdown() and not turtle_herder.is_disabled:
        rospy.sleep(0.3)
    turtle_herder.shutdown()
