#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Description
##############################################################################

"""
.. module:: exceptions
   :platform: Unix
   :synopsis: Exceptions raised by loading/unloading of interactions.


This module defines exceptions raised by the rocon_interactions package.
These exception names are all included in the main
rocon_interactions namespace.  To catch one, import it this
way:

.. code-block:: python

    from rocon_interactions import InvalidInteraction

----

"""

##############################################################################
# Exceptions
##############################################################################


class InvalidInteraction(Exception):
    """
      Whenever an interaction has been specified incorrectly.
    """


class YamlResourceNotFoundException(IOError):
    """
      The requested yaml resource could not be found.
    """


class MalformedInteractionsYaml(Exception):
    """
      Whenever malformed yaml is used in loading a set of interactions.
    """


class FailedToStartRappError(Exception):
    """ Failed to start rapp. """
    pass


class FailedToStopRappError(Exception):
    """ Failed to stop rapp. """
    pass

class FailedToListRappsError(Exception):
    """ Failed to list rapps. """
    pass
