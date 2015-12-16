Rostful
=======

This `Python package`_ and `ROS`_ package allows sending `REST`_ request to a multiprocess system supported by `Pyros`_.

ROS usage
---------

Rostful interfaces a `ROS`_ system and the web world through a `REST`_ API.
A `ROS name` ::

  /namespace/node_name/service_name

is made available via the URL (by default) ::

  http://localhost:8000/ros/namespace/node_name/service_name

- A service accept a POST request and returns a json message containing the original ROS service response
- A Topic accept a POST on a Subscriber ( and returns nothing ) and a GET on a Publisher (and returns the last message received from that publisher)
- A Param accept GET and POST request to get/set the value.

Errors:

- A request that is successful but doesnt return anything ( publisher didn't send any message ) return 204
- A request to an non existent topic or service returns 404
- A request with wrong message format returns 400
- A request that triggers an error in the ROS system returns 500, as well as a traceback, usually very handy for debugging the issue.
- A request that is not replied in 10 seconds returns 504



.. include:: weblinks.rst

Contents:

.. toctree::
   :maxdepth: 2

   readme_link
   internals
   changelog_link

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
