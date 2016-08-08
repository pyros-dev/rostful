.. image:: https://travis-ci.org/asmodehn/rostful.svg?branch=indigo-devel
    :target: https://travis-ci.org/asmodehn/rostful

Overview
========

ROStful - A REST API for ROS.

This repository has a few main branches:

- master : main branch, python dev workflow, releasing version tags into a pip package.
- indigo-devel : current indigo-based ongoing development. catkin dev workflow.
- indigo : current indigo-based release (ROS pkg - tags attempting to match)
- <ros_distro> : current <ros_distro>-based release (ROS pkg)

Apart from these we follow a `feature branching workflow <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow>`_

WARNING: This repository structure is currently being implemented...

Rostful is intended to be the outside layer of a ros system. meaning it will interface other software systems with ros.
As such this should be launched either :
* as a python code with the de facto python standard behaviors ( venv, pip requirements, etc. ),

```
python -m rostful flask
```

* as a ros package, with the de facto ros standard behaviors.

```
roslaunch rostful rostful.launch
```

so that users from both world can use it efficiently.


PYTHON VIRTUALENV SETUP
=======================

How to setup your python virtual environment on Ubuntu (tested on Trusty 14.04)
* Install and Setup virtualenvwrapper if needed
```
sudo apt-get install virtualenvwrapper
echo "source /etc/bash_completion.d/virtualenvwrapper" >> ~/.bashrc
```
* Create your virtual environment for your project
```
mkvirtualenv myproject --no-site-packages
workon myproject
```
* Populate it to use rostful. The catkin dependency is temporarily needed to be able to use the setup.py currently provided in rostful.
```
pip install catkin-pkg rostful
```

|Build Status| # ROSTful

Try it now
----------

Go check the `examples`_

 Overview
---------

ROStful is a lightweight web server for making ROS services, topics, and
actions available as RESTful web services. It also provides a client
proxy to expose a web service locally over ROS.

ROStful web services primarily use the `rosbridge`_ JSON mapping for ROS
messages. However, binary serialized ROS messages can be used to
increase performance.

The purpose of ROStful is different from `rosbridge`_: rosbridge
provides an API for ROS through JSON using web sockets. ROStful allows
specific services, topics, and actions to be provided as web services
(using plain get and post requests) without exposing underlying ROS
concepts. The ROStful client, however, additionally provides a modicum
of multi-master functionality. The client proxy is a node that connects
to a ROStful web service and exposes the services, topics, and actions
locally over ROS.

The ROStful server has no dependencies on 3rd party libraries, and is
WSGI-compatible and can therefore be used with most web servers like
Apache and IIS.

 ROStful web services
~~~~~~~~~~~~~~~~~~~~~

A ROStful web service is a web service that uses ROS data structures for
input and output. These include services, topics, and actions.

Service methods accept as input a ROS message over HTTP POST and return
a ROS message in the response. The input and output cannot be defined
directly with ROS messages; it must use a ROS service definition.

Methods denoted as topics may use any ROS message, but are limited to
accepting that message via HTTP POST or returning it, taking no input,
via HTTP GET. Topic methods do not need to allow both methods. A topic
method allowing POST is described as a “subscribing” method, and one
that allows GET is a “publishing” method.

Methods denoted as actions consist of a set of subsidiary topic methods,
the subscribing-only ``goal`` and ``cancel`` methods, and the
publishing-only ``status``, ``result``, and ``feedback`` methods. These
methods are located at the url ``<action  method url>/<suffix>``, where
the suffix is the subsidiary method name.

The ROStful server
~~~~~~~~~~~~~~~~~~

The ROStful server can provide services, topics, and actions that are
locally available over ROS as ROStful web services. Topics may be
specified as publishing, subscribing, or both.

ROStful uses the rosbridge JSON mapping by default, but binary
serialized ROS messages can be sent with the ``Content-Header`` set to
``application/vnd.ros.msg``. Giving this MIME type in the ``Accept``
header for queries without input (i.e., publishing topic methods) will
cause the server to return serialized messages.

 The ROStful client
~~~~~~~~~~~~~~~~~~~

The ROStful client is a node that connects to a ROStful web service and
makes its services, topics, a

.. _examples: https://github.com/asmodehn/rostful/tree/indigo-devel/rostful_examples
.. _rosbridge: http://wiki.ros.org/rosbridge_suite

.. |Build Status| image:: https://travis-ci.org/asmodehn/rostful.svg?branch=indigo-devel
   :target: https://travis-ci.org/asmodehn/rostful