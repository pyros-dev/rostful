#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_python_utils
import rospkg
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Methods
##############################################################################


def setup_home_dirs():
    if not os.path.isdir(get_home()):
        os.makedirs(get_home())
    if not os.path.isdir(get_icon_cache_home()):
        os.makedirs(get_icon_cache_home())
    if not os.path.isdir(get_settings_cache_home()):
        os.makedirs(get_settings_cache_home())


def get_home():
    '''
      Retrieve the location of the home directory for the rocon remocon's
      temporary storage needs

      @return the rocon remocon home directory (path object).
      @type str
    '''
    return os.path.join(rospkg.get_ros_home(), 'rocon', 'remocon')


def get_icon_cache_home():
    '''
      Retrieve the location of the directory used for storing icons.

      @return the rocon remocon icons directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'icons')


def get_settings_cache_home():
    '''
      Retrieve the location of the directory used for storing qt settings.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'cache')


def find_rocon_remocon_script(name):
    """
    Get the path to the internal script of the specified name. Note that this changes
    depending on whether you are working in a devel or an install space. Let the
    find resource handler discover where they are.

    :returns: full absolute pathnmae to the script
    :rtype: path
    :raises: `rospgk.ResourceNotFound`
    """
    return rocon_python_utils.ros.find_resource('rocon_remocon', name)


def get_web_browser():
    """
    Do a search through preferred browsers which most importantly can handle
    web apps and return the path to their executables.

    :returns: pathname to the browser
    :rtype: str
    """
    if rocon_python_utils.system.which("google-chrome"):
        return 'google-chrome'
    elif rocon_python_utils.system.which("google-chrome-unstable"):
        return 'google-chrome-unstable'
    elif rocon_python_utils.system.which("chromium-browser"):
        return 'chromium-browser'
    return None

def get_web_browser_codename():
    """
    returns available browsers codename

    :returns:  web browser code name
    :rtype: str
    """
    # Currently it only supports chrome
    return rocon_std_msgs.Strings.OS_CHROME
