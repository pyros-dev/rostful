================================================
Rostful Tutorial 1 : Chatter - the roslaunch way
================================================

*******
Install
*******

Create workspace for rostful testing::

  mkdir -p rostful_ws/src
  cd rostful_ws/src

You can manage your workspace as usual with wstool (optional)::

    wstool init
    wstool set rostful ...
    wstool set pyros ...

Install dependencies::

    rosdep install --from-path src --ignore-src -y

**Is there a Problem ?**

- Currently tornado might not be installed and the debian package from trusty is too old.
  you need tornado > 4.0, so on trusty you should install it from pip::

    sudo apt-get install pip
    sudo pip install tornado


*****
Build
*****

Build the workspace as usual with catkin::

    source /opt/ros/indigo/setup.bash
    catkin_make

**Is there a Problem ?**

***
Run
***

Start roscore in one terminal::

    source /opt/ros/indigo/setup.bash
    roscore &

Start a talker node in another terminal::

    source /opt/ros/indigo/setup.bash
    rosrun roscpp_tutorials talker


Start rostful in a third terminal::
 
    source devel/setup.bash
    roslaunch rostful rostful.launch


To understand what is happening, it is always good to have a look at your system.
Here are a few commands useful to inspect::

    yujin@lenovo:~/rostful_ws$ rosnode list
    /rosout
    /rostful
    /talker

    yujin@lenovo:~/rostful_ws$ ps aux | grep rostful
    yujin     6617  1.0  0.2 287180 21620 pts/9    Sl+  12:40   0:00 /usr/bin/python /opt/ros/indigo/bin/roslaunch rostful rostful.launch
    yujin     6635  0.0  0.0  23636  3512 ?        Ss   12:40   0:00 /bin/bash /home/yujin/rostful_ws/src/rostful/scripts/devserver -p 8080 -s flask ~connections_list:=/rocon/connection_cache/list ~connections_diff:=/rocon/connection_cache/diff __name:=rostful __log:=/home/yujin/.ros/log/f8e4d7c4-4a3a-11e6-810e-5c514fba7886/rostful-1.log
    yujin     6639  1.0  0.5 159696 41840 ?        S    12:40   0:00 python -tt -m rostful run -p 8080 -s flask --ros-arg=~connections_list:=/rocon/connection_cache/list --ros-arg=~connections_diff:=/rocon/connection_cache/diff --ros-arg=__name:=rostful --ros-arg=__log:=/home/yujin/.ros/log/f8e4d7c4-4a3a-11e6-810e-5c514fba7886/rostful-1.log

    yujin@lenovo:~/rostful_ws$ pstree -cap 6617
    roslaunch,6617 /opt/ros/indigo/bin/roslaunch rostful rostful.launch
    ├─devserver,6635 /home/yujin/rostful_ws/src/rostful/scripts/devserver -p 8080 -s flask...
    │   └─python,6639 -tt -m rostful run -p 8080 -s flask ...
    │       ├─python,6642 -tt -m rostful run -p 8080 -s flask ...
    │       │   ├─{python},6690
    │       │   └─{python},6718
    │       ├─python,6661 -tt -m rostful run -p 8080 -s flask ...
    │       └─python,6688 -tt -m rostful run -p 8080 -s flask ...
    │           ├─{python},6704
    │           ├─{python},6705
    │           ├─{python},6708
    │           ├─{python},6716
    │           ├─{python},6717
    │           └─{python},6720
    ├─{roslaunch},6624
    └─{roslaunch},6625


    yujin@lenovo:~/rostful_ws$ rostopic list
    /chatter
    /rosout
    /rosout_agg


Change rostful Config and Restart::

    <arg name="topics" default="['/chatter']" />

Open your browser http://localhost:8080, you should see the Chatter topic   


**Is there a Problem ?**

- If you don't see anything on the webpage, there is probably a mistake in the configuration.
  Luckily you can pass a regular expression, so change rostful config like so, and restart::

    <arg name="topics" default="['/.*']" />
    <arg name="services" default="['/.*']" />


