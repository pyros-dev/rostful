[![Build Status](https://travis-ci.org/asmodehn/rostful.svg?branch=indigo-devel)](https://travis-ci.org/asmodehn/rostful)

Overview
========

ROStful - A REST API for ROS.

Rostful is intended to be the outside layer of a ros system,
communicating with the outside world via HTTP,
and exposing a REST API to use the robot services, or introspect robot topics.
As such this should be launched either : 
 - as a python code with the de facto python standard behaviors ( venv, pip requirements, etc. ),

``` python -m rostful flask ```

 - as a ros package, with the de facto ros standard behaviors.

``` roslaunch rostful rostful.launch ```

so that users from both world can use it efficiently.

PYTHON VIRTUALENV SETUP
=======================

How to setup your python virtual environment on Ubuntu (tested on Trusty
14.04)
 - Install and Setup virtualenvwrapper if needed
``` sudo apt-get install virtualenvwrapper echo "source /etc/bash_completion.d/virtualenvwrapper" >> ~/.bashrc ```

 - Create your virtual environment for your project
``` mkvirtualenv myproject --no-site-packages workon myproject ```

 - Populate it to use rostful. The catkin dependency is temporarily needed
to be able to use the setup.py currently provided in rostful.
``` pip install catkin-pkg rostful ```