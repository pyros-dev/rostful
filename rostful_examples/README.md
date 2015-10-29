# ROSTful Examples
Basic examples for Rostful

## Installation

### From Source

First, install [ROS Indigo release](http://wiki.ros.org/indigo/Installation/Ubuntu).

Then, make sure the ROS_DISTRO environment variable is set correctly:
```
echo $ROS_DISTRO
```

It may already be.  If not, issue this shell command:
```
$ export ROS_DISTRO=indigo
```

Then, clone the source repositories:

```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ wstool init src https://raw.githubusercontent.com/asmodehn/rostful/indigo-devel/rosinstall/$ROS_DISTRO.rosinstall
```

Install all dependencies:
```
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
```

Then, build everything:
```
$ catkin_make
$ source devel/setup.bash
```

## Test :
```
$ roslaunch rostful_examples turtlesim_dev.launch --screen
```

Then use [Postman](https://chrome.google.com/webstore/detail/postman-rest-client/fdmmgilgnpjigdojojpjoooidkmcomcm?hl=en)/Chrome or [HTTPRequester](https://addons.mozilla.org/en-us/firefox/addon/httprequester/)/Firefox or any other REST client extension for your favorite browser to be able to manually send requests to the exposed ROS topics/services
Use your hostname ( localhost on your local machine where you probably run turtlesim )
The port where the webserver listens by default is 8080 for security reasons (ports < 1024 require administrator rights)
Then we append a /ros/ prefix to form the URL from the ROS name. This allows us to leave URL space for other pages we might want to serve.

So for turtlesim you should be able to get the current position at this URL:
GET http://localhost:8080/ros/turtle1/pose

This is still work in progress, so more details will be documented here when this gets finalized.
