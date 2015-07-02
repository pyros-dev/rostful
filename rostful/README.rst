Overview
========

ROStful - A REST API for ROS.

Rstful is intended to be the outside layer of a ros system. meaning it will interface other software systems with ros.
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

TODO
====

Because of the many python dependencies that may not be suitable/possible to expose via rosdep, this package should slowly become a pure python package.
Rostful-node will be the package handling the transition from ros world to python world.


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

