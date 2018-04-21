|Build Status| |Documentation Status| |Updates| |Python 3|

ROSTful
=======

ROStful - A REST API for ROS.

We follow a `feature branching workflow <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow>`_

Rostful is intended to be the outside layer of ROS (and eventualy any multiprocess system). Meaning it will interface ROS with other systems, over the internet, via HTTP and through a REST API.
As such, this should be used as any python program, from a virtual environment. We also need to install the specific pyros interface for the system we want to expose.

```
$ mkvirtualenv rostful
(rostful)$ pip install rostful pyros[ros]
(rostful)$ python -m rostful flask
```

A ROS package is provided as a third party release, for ease of deployment within a ROS system.
However it is heavily recommended to do development the "python way", dynamically, using virtual environments, with quick iterations.
The rostful PyPI package is released more often than the ROS package, and will have latest updates available for use.


PYTHON VIRTUALENV SETUP
=======================

How to setup your python virtual environment on Ubuntu (tested on Xenial 16.04)
* Install and Setup virtualenvwrapper if needed
```
sudo apt install virtualenvwrapper
```
* Create your virtual environment for your project
```
$ mkvirtualenv myproject
```
* Populate it to use rostful. The catkin dependency is temporarily needed to be able to use the setup.py currently provided in rostful.
```
(myproject)$ pip install rostful pyros[ros]
(myproject)$ pip install rostful
```


Try it now
----------

Go check the `examples`_

 Overview
---------

ROStful is a lightweight web server for making ROS services, topics, and
actions available as RESTful web services.

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



What will not be in Rostful
===========================
 - Security related stuff ( Authentication/Authorization ) implementation.
 We will not provide here any Authentication/Authorization mechanisms without ROS providing one first.
 And even after that, the implications of such an implementation would probably fit better in another specific microservice, that we would rely on in rostful.



.. _examples: https://github.com/asmodehn/rostful/tree/indigo-devel/rostful_examples
.. _rosbridge: http://wiki.ros.org/rosbridge_suite

.. |Build Status| image:: https://travis-ci.org/asmodehn/rostful.svg?branch=master
   :target: https://travis-ci.org/asmodehn/rostful
   :alt: Build Status

.. |Documentation Status| image:: https://readthedocs.org/projects/rostful/badge/?version=latest
   :target: http://rostful.readthedocs.io/en/latest/?badge=latest
   :alt: Documentation Status

.. |Updates| image:: https://pyup.io/repos/github/asmodehn/rostful/shield.svg
    :target: https://pyup.io/repos/github/asmodehn/rostful/
    :alt: Updates

.. |Python 3| image:: https://pyup.io/repos/github/asmodehn/rostful/python-3-shield.svg
     :target: https://pyup.io/repos/github/asmodehn/rostful/
     :alt: Python 3