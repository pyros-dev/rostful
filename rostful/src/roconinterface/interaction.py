#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions
   :platform: Unix
   :synopsis: Representative class and methods for an *interaction*.


This module defines a class and methods that represent the core of what
an interaction is for a remocon.

----

"""
##############################################################################
# Imports
##############################################################################

import os
import rocon_console.console as console
#import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interactions
from . import utils

##############################################################################
# Classes
##############################################################################


class Interaction(rocon_interactions.Interaction):
    '''
      This class defines an interaction for the rocon_remocon.
      It does so by wrapping the base
      rocon_interaction.Interaction class with a few extra
      variables and methods.
    '''
    __slots__ = [
        'launch_list',  # dictionary of launch information
        'icon',
        'index'
    ]

    def __init__(self, msg):
        """
          :param msg: underlying data structure with fields minimally filled via :func:`.load_msgs_from_yaml_resource`.
          :type msg: rocon_interaction_msgs.Interaction_

          .. include:: weblinks.rst
        """
        super(Interaction, self).__init__(msg)
        self.launch_list = {}
        self.index = None
        icon_name = msg.icon.resource_name.split('/').pop()
        if msg.icon.data:
            icon = open(os.path.join(utils.get_icon_cache_home(), icon_name), 'w')
            icon.write(msg.icon.data)
            icon.close()
        self.icon = icon_name

    def __str__(self):
        '''
          Format the interaction into a human-readable string.
        '''
        s = rocon_interactions.Interaction.__str__(self)
        s += console.cyan + "  Launch List" + console.reset + "  : " + console.yellow + "%s" % self.launch_list.keys() + console.reset + '\n'  # noqa
        return s
