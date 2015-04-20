ROStful
=======

[![Join the chat at https://gitter.im/asmodehn/rostful](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/asmodehn/rostful?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

ROStful is a lightweight web server for making ROS services, topics, and actions available as RESTful web services. It also provides a client proxy to expose a web service locally over ROS.

ROStful web services primarily use the [rosbridge](http://wiki.ros.org/rosbridge_suite) JSON mapping for ROS messages. However, binary serialized ROS messages can be used to increase performance.

The purpose of ROStful is different from [rosbridge](http://wiki.ros.org/rosbridge_suite): rosbridge provides an API for ROS through JSON using web sockets. ROStful allows specific services, topics, and actions to be provided as web services (using plain get and post requests) without exposing underlying ROS concepts.
The ROStful client, however, additionally provides a modicum of multi-master functionality. The client proxy is a node that connects to a ROStful web service and exposes the services, topics, and actions locally over ROS.

The ROStful server has no dependencies on 3rd party libraries, and is WSGI-compatible and can therefore be used with most web servers like Apache and IIS.

ROStful web services
--------------------

A ROStful web service is a web service that uses ROS data structures for input and output. These include services, topics, and actions.

Service methods accept as input a ROS message over HTTP POST and return a ROS message in the response. The input and output cannot be defined directly with ROS messages; it must use a ROS service definition.

Methods denoted as topics may use any ROS message, but are limited to accepting that message via HTTP POST or returning it, taking no input, via HTTP GET. Topic methods do not need to allow both methods. A topic method allowing POST is described as a "subscribing" method, and one that allows GET is a "publishing" method.

Methods denoted as actions consist of a set of subsidiary topic methods, the subscribing-only `goal` and `cancel` methods, and the publishing-only `status`, `result`, and `feedback` methods. These methods are located at the url `<action  method url>/<suffix>`, where the suffix is the subsidiary method name.

The ROStful server
------------------
The ROStful server can provide services, topics, and actions that are locally available over ROS as ROStful web services. Topics may be specified as publishing, subscribing, or both.

ROStful uses the rosbridge JSON mapping by default, but binary serialized ROS messages can be sent with the `Content-Header` set to `application/vnd.ros.msg`. Giving this MIME type in the `Accept` header for queries without input (i.e., publishing topic methods) will cause the server to return serialized messages.

The ROStful client
------------------
The ROStful client is a node that connects to a ROStful web service and makes its services, topics, and actions locally available over ROS.

The client can be given the root URL of a ROStful web service, in which case it will connect to all services, topics, and actions for that web service, or it can be given the URL of an individual service, topic, or action to connect to.
When connecting directly, the name of service, topic, or action is used as the service, topic, or action name. When connecting to the root URL, the name of the web service, if any, is used as a prefix (followed by a slash) to the service/topic/action names. This can be controlled using the `--prefix` and `--no-prefix` options.

For privacy, the client will not allow the web service to subscribe to local ROS topics unless the `--allow-subscription` option is given.

The client periodically queries the server for its published topics, and then publishes those messages locally. The default interval is 1 second, but this can be set with the `--publish-interval` option.

The `--binary` option directs the client to use binary serialized messages instead of JSON.

For debugging, the client can be used with the same ROS master as the server. Giving the `--test` option causes the client to append `_ws` to the names of the services/topics/actions it connects to, so they don't conflict with those being used by the server.

Web service/component description format
========================================
The ROStful server functions as a ROS component, providing services and actions, and publishing and subscribing to topics. Due to the lack of a standard format for describing ROS components, ROStful uses simple INI-file-based format. The file is broken up into sections, with each section starting with a header contained in square brackets. In addition to standard INI file formatted sections, "definition" sections are allowed for defining messages, services, and actions.

Manifest section
----------------
There is a single optional section titled `Manifest`. The section is in standard INI file format, a sequence of lines each containing a key-value pair separated by `=`. The special key `Def-Type` indicates the type of description being provided (e.g., `Node`, `Service`, `Topic`, `Action`).

For services, topics, and actions, the keys `Type` and `Name` are given, which give the ROS type of the service/topic/action, and the service/action name or topic as appropriate.

For topics, the keys `Publishes` and `Subscribes` are given with the values `true` or `false`.

Other sections
--------------
The `Node` description includes the optional sections `Services`, `Actions`, `Publishes`, and `Subscribes`. These sections also follow the INI file format, where the keys are the names of the services, actions, or topics, and the values are their ROS types.

Definitions
-----------
Definition sections have a title in the form `<definition type>:<definition name>`. The definition type, given before the colon, can be one of `msg`, `srv`, or `action`, corresponding to ROS message, service, and action definitions. The definition name, given after the colon, is the name of the data structure being defined. The content of the section is exactly the format used in the ROS files defining messages, services, and actions, respectively.

Purpose
-------
This description format allows the full definition for a component to be given inline. The description for a node or method is located at `<node or method url>/_rosdef`. Setting the query parameter `full=true` will include all the definitions referenced in description (recursively). Setting the query parameter `json=true` will return a JSON representation of the definition.
