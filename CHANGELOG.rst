Changelog
=========



0.2.1 (2018-04-22)
------------------
- Now storing log in ROS_HOME when running on ROS as installed package.
  cosmetics. [AlexV]


0.2.0 (2018-04-21)
------------------
- V0.2.0. [AlexV]
- Integrated release commands to setup.py. [AlexV]
- Cleaning up param interface to only keep a working minimum view.
  [AlexV]
- Fixing getting param value. [AlexV]
- Removing unused scripts. __main__ is now the entrypoint. [AlexV]
- Pin numpy to latest version 1.14.2. [pyup-bot]
- Pin hypothesis to latest version 3.56.3. [pyup-bot]
- Pin pytest-xdist to latest version 1.22.2. [pyup-bot]
- Pin pytest to latest version 3.5.0. [pyup-bot]
- Removing outdated ros content. has already been moved to examples or
  pyros interface. [AlexV]
- Adding pyup badges. [AlexV]
- Cleaning up Readmes. added gitchangelog and pyup configuration.
  [AlexV]
- Updating sphinx config. [AlexV]
- Fixing dependencies to latest pyros. [AlexV]
- Reverting to latest tox. [AlexV]
- Reviewing dependencies and ROS packages versions. [AlexV]
- Cleaning up travis matrix in yml. fixing travis-env in tox.ini.
  [AlexV]
- Pinning tox to 2.9.1 for travis testing. [AlexV]
- Quick fix to install python-catkin-pip on travis (doesn't get
  automatically pulled by rosdep apparently) [alexv]
- Adding pyros dependency. [AlexV]
- Fixing imports for new pyros_04. forcing PyrosROS node. removed unused
  server module. cleaning up instance config file. [alexv]
- Migrating to latest catkin_pip. [AlexV]
- Readme file command correction. [Thomas]

  Fixed the command to Install and setup the virtualenvwrapper
- Not upgrading tox on travis just yet. [AlexV]
- Fixing tox params. [AlexV]
- Removing tox options for hypothesis. [AlexV]
- Removing ROS specific files. adding tox and requirements. [AlexV]
- Fixing python-flask-restful dependency. [AlexV]
- Adding kinetic travis checks. [AlexV]
- Preventing switch to tornado 5 for now (breaking install on lder
  systems). [AlexV]
- Making dependency on backports.ssl-mach-hostname system version
  dependent. [AlexV]
- Adding python install dependency on backports.ssl-match-hostname on
  python <=3.4. [AlexV]
- Now travis install current package in venv before running tests to
  make sure we retrieve dependencies. [AlexV]
- Removing submodules from setup.py packages_dir. they re gone. [AlexV]
- Default config exposes everything. Better for ease of use for now...
  [AlexV]
- Commenting useless stuff. [AlexV]
- Getting flask restful resource to work. [AlexV]
- Getting ROS interface by defaut. better for now. [AlexV]
- Cleaning up setup.py. [alexv]
- Now relying on catkin_pip 0.2.3. [alexv]
- Changing nose -> pytest comments about package_data. [alexv]
- Removing jade as it is EOL and we are missing dependencies. commenting
  kinetic as upgrading dependencies from ROS package to system package
  is not working yet : https://github.com/ros-
  infrastructure/rosdep/issues/539. [alexv]
- Fixing package data for templates. [alexv]
- Removing git submodules to now rely on python dependencies. [alexv]
- Fixing package-data description for jquery-mobile. [alexv]
- Adding tblib dependency since we use it directly. [alexv]
- Hack to work around potential catkin_pip bug. [alexv]
- Fixing tests on xenial with pytests. [alexv]
- Now using catkin_pip 0.2. [AlexV]
- Fixing tests with pyros 0.4. [AlexV]
- Comments and requirements changes. [AlexV]
- Fixing import to work with pyros_04. [AlexV]
- Change branch name for pyros. [Thomas]

  Changed pyros branch used in indigo-devel.rosinstall
  as the specified branch for pyros (eg: indigo-devel) no longer
  exist.
- Adding flask_restful dependency for ROS build. adding pyros_config
  dependency for python setup. [alexv]
- Added flask_reverse_proxy for ROS and pyros_setup for python. [alexv]
- Ddign flask_cors dependency for ros and fixing pyros version for
  python. [alexv]
- Changes to use catkin_pip with setuptools. [alexv]
- Removed static data from manifest.in, to match pyros manifest, hoping
  to fix travis. [alexv]
- Adding detailed description of all package data we want, both in
  setup.py and Manifest.in. [alexv]
- Fixed some dependencies. improves how to find the configuration file
  or generate if needed. [alexv]
- Fixing how we determine instance_path in ROS usecase. adding
  simplejson dependency for python. [alexv]
- Temporary declaring a dependency on python-mock, because of pyros
  0.3.0. [alexv]
- Adding roslint dependency to build with current CMakeLists. [alexv]
- Removing passlib and rester from dependencies list. not needed now.
  [alexv]
- Upgrading to package.xml v2, updating travis script and fixing tests.
  [alexv]


0.1.0 (2017-01-13)
------------------
- Releasing 0.1.0 with catkin. [alexv]
- Revert "using tblib 1.2 from ros TPR package" [alexv]

  This reverts commit cf1a9c8989a778d4a1096c45f3cb9960e1727c05.
- Changed default connection cache topics. comments. [alexv]
- Resolving import name collision. [AlexV]
- Using tblib 1.2 from ros TPR package. [alexv]
- Now also installing blueprints. [alexv]
- Fixing urls behind proxy. simplified routing. [alexv]
- Adding comment about using apispec to generate swagger spec. [alexv]
- Python cosmetics. [alexv]
- Fixing nopyrosclient test and various python issues. [alexv]
- Adding logformatter. [alexv]
- Now using blueprint to be able to hold multiple version of API. lost
  of structure changes and simplifications. [alexv]
- Now passing logfile and config from roslaunch to the app. [alexv]
- Now explicitely setting the instance path to ros home. especially
  useful when using from install space. [alexv]
- Catching exception when we cannot find rostful.cfg. [alexv]
- Adding tblib as a ros dependency. [alexv]
- Revert to catkin build until all dependencies are released to get
  install space to work. [alexv]
- Adding flask-reverse-proxy as dependency in setup.py. [alexv]
- Readding submodules to setup.py until we get thirdparty releases in.
  [alexv]
- Now rostful listening on all IPs. for now. unsecure but easier to use
  out of the box. [alexv]
- Describing improved repository structure. [alexv]
- Adding pyros pypi package version. [AlexV]
- Adding bwcompat to be able to use old pyros as well. [alexv]
- Improving configuration loading and overloading. added /help to list
  avialable URLs. stop using SERVER_NAME : changes URL generation, and
  strange problems appear... [alexv]
- Fixes (some are tmp) to run with current pyros config_refactor branch
  and catkin_pip. [alexv]
- Added requirements to build from source with catkin_pure_python.
  [alexv]
- Removed now useless flask_login link. [alexv]
- Improving command line behavior, regarding default arguments and
  config file. [alexv]
- Added configuration file. added version number. cleanup to run as pure
  python package. [alexv]
- Removing obsolete install rules. setup.py should manage this now.
  [alexv]
- Now using catkin_pure_python. [alexv]


0.0.10 (2016-08-25)
-------------------
- Releasing 0.0.10 for gopher benevolent. [alexv]
- Update tutorial.rst. [AlexV]
- Adding talker tutorial. [Marcus Liebhardt]
- Removed test link from template. [alexv]
- Allowing rostful to run behind a reverse proxy. [alexv]
- Now changing NaN to null in every response. cosmetics. [alexv]
- Using simplejson to be able to change nan from python to null in json,
  since NaN is not valid json. [alexv]
- Quick fix service and topic type introspection. [alexv]
- Fixing GET request to backend ros services (disabling full and json
  arguments, parser seems somehow broken) fixing GET request with
  _rosdef suffix. [alexv]
- Cleanup while unifying code design with task planner. [alexv]
- Removed import of flask_security, which we don't use anymore. [alexv]
- Adding support for message type Header  in template. [alexv]
- Fixing install rules. cleanup migrations. [alexv]
- Cleaning up obsolete arguments from ros launcher. [alexv]
- Removind security / login / db form here. probably the wrong place and
  time to try that. [alexv]
- Fix issue with leading "/" not being added to the rosname we get from
  url. fixing all tests to pass, een if they are still empty. [alexv]
- Fixed basic app tests with and without pyros connection. [alexv]
- Improved design following flask documentation. but templates not found
  for tests. [alexv]
- Extracted wsgi app from server. started wsgi unittest with testapp
  working. much more work to be done. cleaning up and cosmetics. [alexv]
- Cleanup in progress, mostly working. wait for tests before merging
  into main branch. [alexv]
- Adding sqlalchemy as dependency. [alexv]
- Adding python-wtforms as ros dependency. [alexv]

  Conflicts:
  	rostful/deps/testfixtures
- Adding python-wtforms as ros dependency. [alexv]
- New connection cache feature disabled by default. [AlexV]
- Adding doc for cache topics args. [alexv]
- Now passing basepath to pyros context for ros dynamic setup if needed.
  [alexv]
- Adding option to enable cache or not from rosparams. [alexv]
- Adding remap arguments, useful if using connection_cache. [alexv]


0.0.9 (2016-01-28)
------------------
- Attempting travis fix. [alexv]
- A beginning of documentation, and getting ready for 0.1 release...
  [alexv]
- Handling service timeout and not found as exception to return correct
  error status. [alexv]
- Fixing rester test for topics. now passing. [alexv]
- Fixed http response for mute publishers.  added rester tests for
  topics.  cosmetics. [alexv]
- Adding rester tests to be run with rostest to verify rostful behavior
  with pyros testnodes. fixing roster script to strip useless rosargs
  from rostest run. [alexv]
- Small refactoring to make testing rostful easier. [alexv]
- Fixing pyrosexception import. [alexv]
- Cosmetics. [alexv]
- Improving exception catching and fowarding to web client. [alexv]
- Removing check for allow_pub / allow_sub. authorization should not be
  done here. [alexv]
- Fixed for changes to pyros version 0.1. [alexv]
- Starting to get topics and services list with pyros 0.1. [alexv]
- Fixing rosinstall files given new workspace structure for examples.
  [alexv]
- Bumping reverted flask-restful. [alexv]
- Bumping modified passlib and flask-restful. [alexv]
- Adding all dependencies as submodules and getting rostful to work
  again without flask-ext-catkin. [alexv]
- Removing examples, since they are now in a separate repository.
  [alexv]
- Adding a lot of dependencies from flask-ext-catkin. [alexv]
- Reorganizing documentation. [alexv]
- Merging old markdown doc into RST doc. [alexv]
- Removed useless mercurial file. [alexv]
- Moving everything one folder down. [alexv]
- Moved examples. removed src/. fixed setup.py. [alexv]
- Commenting rester package and dependencies as they make problems on
  build at the moment. [alexv]
- Adding symlink in src to workaround catkin < 0.6.15 package_dir issue.
  [alexv]
- Fixing roslaunch instructions. [AlexV]
- Removed obsolete sample code. [alexv]
- Bumping rester to be able to call apirunner form python. [alexv]
- Fixing response to set content-type properly. [alexv]
- Adding bool implementation for msg params in frontend. [alexv]
- Handle empty request properly now. [alexv]
- Adding content_type if service returns None. [alexv]
- Fixing setup.py for install. [alexv]
- Adding travis badge. [alexv]
- Fixing travis build, only for rostful package. [alexv]
- Starting travis integration. [alexv]
- Adding Rester for tests. fixed content type on backend. first tests
  working. [alexv]
- Integrating params. backend has been tested. frontend not there yet.
  [alexv]


0.0.8 (2015-10-10)
------------------
- 0.0.8. [Daniel Stonier]
- Removing verbose logging with tornado. [alexv]
- Removed import in the middle of the file. [alexv]
- Now importing logging handlers in server. [alexv]
- Moving rotating file logging handler setup to launch method to avoid
  creating every time the view is initialized. [alexv]
- Tentative to set default flask logging level. [alexv]
- Cosmetics. [alexv]
- Removed overlogging. [alexv]
- Fixes after api changes. removed useless msgconv here. [alexv]
- Added comment to mark the rocon_interface as broken. [AlexV]
- POST works, but called multiple times on button press. [Michal
  Staniaszek]
- Frontend and backend get working. [Michal Staniaszek]
- Rostful backend converted to use node client. [Michal Staniaszek]
- Replace some direct calls to ros_if with client calls. Can see
  topic/service list. [Michal Staniaszek]
- Reverting to flask for easier debugging. [alexv]
- Small log improvements with tornado. [alexv]
- Cleaning logging abit. switched back to flask because tornado doesnt
  log exceptions. [alexv]
- Adding TODO to get rid of reqparse. [alexv]
- Prepend slashes to the requests to backend. [Michal Staniaszek]
- Adding webworker example, as a mean to introspect running ros
  system... [alexv]
- Removed useless log. [alexv]
- Removing celery worker leftovers and cleaning up. [alexv]
- Ros arguments split and properly read by click. [Michal Staniaszek]

  issue #40
- Better integration with click, flask/tornado option. [Michal
  Staniaszek]

  Using click grouping functionality, can run the various functions from the
  devserver script.

  Can select to use either the flask or tornado webservers, specify in
  rostful.launch.

  Fixes issues #40 and #43.
- Rostful uses click for cli parameters. [Michal Staniaszek]
- Working example for turtlesim. [Michal Staniaszek]
- Update README.md. [Michal Staniaszek]
- Removing celery related code from rostful for simplicity. it has been
  moved to asmodehn/celeros. [alexv]
- Passing a node name to rostful_node to avoid conflict with other
  instances of it. [alexv]
- More favicon fixes. [alexv]
- Adding favicon in layout. [alexv]
- Adding favicon and trying to address performance issues... [alexv]
- Fixing import for RostfulCtx. [alexv]
- Replacing rospy.log by flask logger for wsgi views, so we dont fill
  the rospy log with web stuff. [alexv]
- Removing scheduler backend since we can use flower now :) [AlexV]
- Removing custom schedule API and passing config to flower instead.
  [AlexV]
- Allowing to call a service with GET ( and empty request ). [AlexV]
- Fixes after gopher_rocon changes and flower as source dependency.
  adding moment.js lib. [AlexV]
- Simplifying handling of celery task feedback data. [AlexV]
- Fixing id on submit button for topics. [AlexV]
- Adding flower to the mix... [AlexV]
- Changing ETA to be an ISO string. adding moment.js to our js libs.
  [AlexV]
- Fixing pip requirements. [AlexV]
- Improving release scripts and dependency handling... [AlexV]
- Improving rostful setup.py for working both with ros or without. still
  WIP. [AlexV]
- Adding install rules for migrations interactions and launch files.
  [AlexV]
- Fixing package_data to be installed with catkin_make install. [AlexV]
- Adding gunicorn dependency. [AlexV]
- Adding ETA to schedule api. [AlexV]
- Specifying methods for schedule api. [AlexV]
- Implemented first test version of schedule REST API. using imported
  celery task. [AlexV]
- Dynamically update celery broker url from command line => worker and
  scheduler should be using same config. [AlexV]
- Adding parameter to disable worker, but still pass a broker to
  schedule tasks for others. [AlexV]
- Looping up trying new port if socket error. useful to avoid basic dev
  errors. [AlexV]
- Fixing roslaunch parameter when there is no worker. comments. [AlexV]
- Improving parameters to be able to pass broker url. no broker -> no
  worker. [AlexV]
- Adding python-redis as dependency. [AlexV]
- Cleanup for pypi relewse. cosmetics. [AlexV]
- Adding non python files to package. cosmetics. [AlexV]
- Small cleanup for pypi release. [AlexV]
- Preparing pypi release... [AlexV]
- Adding flask-security custom templates. [AlexV]
- Allowing extra tasks to be added via command line parameter. added
  first basic stub for schedule REST API added basic scheduler script to
  launch flower and beat in background and requirements.txt for pip
  dependencies. [AlexV]
- Generic celery task matching an action. on hold. [AlexV]
- Integrating ros actions. WIP. [AlexV]
- Improved tasks api using new pipe client for rostful-node for topic
  and services. [AlexV]
- Now using relative start_rapp stop_rapp services. [AlexV]
- Added task to start/stop rapp. cosmetics. [AlexV]
- Cosmetics and cleanup. [AlexV]
- Design change for celery worker and tasks. Now talking to Rostful node
  services only ( no topic ) => only one rostful node needed.
  integration fo action as celery task in progress... [AlexV]
- Readding templates after move... [AlexV]
- Merged celery app and flask app. [AlexV]
- Implemented topic_extract topic_inject and service_call as celery
  tasks adding code sample to remote call topic_extract topic_inject and
  service_call. added argument to enable worker. [AlexV]
- Improved structure to manage celery worker and requester better.
  [AlexV]
- Config refactoring now using with statement to manage ros resources
  properly. [AlexV]
- Restructuring and playing with signal handlers... [AlexV]
- Setting up consistent branches in rosinstall files. [AlexV]
- Extracting the ros introspection node from rostful repo. so that other
  software can use it. [AlexV]
- WIP. changing rostful structure to be able to implement a celery
  taskable robot... [AlexV]
- Temp commit of refactoring of flask views. [AlexV]
- Cosmetics. [AlexV]
- Fixing rosdep dependency on flask extension to use catkinized version
  of them from flask-ext-catkin. reenabling very useful flask debugger
  but without reload. cosmetics. [AlexV]
- Adding flask-ext-catkin that holds all flask dependencies. [AlexV]
- Attempt to shutdown properly quickly. not working. [AlexV]
- Improved stability. added launchfile. small fixes. [AlexV]
- Eclipse project files. [Daniel Stonier]
- Forcing init script to work in correct directory. [AlexV]
- Adding flask security template, for modification later. [AlexV]
- Fixing database intialization. added script for running it via rosrun.
  little bit of cleanup. [AlexV]
- Making rocon not mandatory for rostful to run. [AlexV]
- Reducing log noise for topic/services changes. [AlexV]
- Removed rocon_rapps. [AlexV]

  removing rocon_rapps since all rapps are included in app_platform package now.
- Adding chatter_concert demo and exposing late topic
  /conversation/chatter. [AlexV]
- Changed rocon branch to indigo-devel since merge. [AlexV]
- Moved examples files out of core rostful package. [AlexV]
- Small webpages layout improvement. [AlexV]
- Iproving webpages. less reloads needed while navigating. [AlexV]
- Fixes for when enable_rocon is false in rostful. [AlexV]
- Added parameter to force enabling rocon features even if specific
  arguments not specified. [AlexV]
- Splitting readme for examples tryout. now hiding empty sections of
  rostful web interface. [AlexV]
- Moved example codes/files out of rostful package. made new package
  rostful_examples. [AlexV]
- Switching all changes for rostful example to specific gocart branches.
  [AlexV]
- Now detecting interactions and exposing rapp public interface. [AlexV]
- Adding sampe webpae for stroll interaction. adding CORS to flask app.
  adding async calls to watcher (unused yet) [AlexV]
- Moved rappwatcher into interactions and using it from there. added
  interaction watcher. added rostful backend rest services. [AlexV]
- Cleanups. added possibility to investigate rapps namespaces with
  rocon. adding interactions in progress... [AlexV]
- Adding fullname to normalize naming of topics/services/actions.
  [AlexV]
- Listing interactions. [AlexV]
- Fixing parameter passing in shell script. [AlexV]
- Simplifying ros_interface. [AlexV]
- Refactoring. adding flask-security for login and flask-migrate for db
  management. making the interface with ros, and the config a separate
  python package. [AlexV]
- Fixing templates to not send arrays with empty params. cosmetics (
  rapps -> interactions ) [AlexV]
- Fixing web templates to support arrays as parameter type. setup to
  access interactions and rosapi services. [AlexV]
- Reenabling other turtle services. [AlexV]
- Adding parameter for zeroconf in turtle_herder. fixing whitelist
  parameter to authorize our concert name. [AlexV]
- Trying to setup turtle_concert tutorial as sample for test - without
  zeroconf. Not working yet ( zeroconf node still gets created ) [AlexV]
- Fixing dependencies to include python-flask-restful from pip. added in
  rosdep. [AlexV]
- Fixing navbar highlight. [AlexV]
- Cleaning up templates to use macros. fixed cancel action. [AlexV]
- Fixing actions in rostful. we need to connect to the server action
  topics. [AlexV]
- Implementing frontend and testing backend for actions. not fully
  working yet. [AlexV]
- Separated ros interface to different module. improved server launch
  and flask templates. [AlexV]
- Adding rostful interaction definition. [AlexV]
- Small changes to startup with gunicorn. not fully getting all
  parameters yet. [AlexV]
- Small cleanup. now generating frontend pages for services as well.
  [AlexV]
- Parameter server and dynamc reconfigure working. [AlexV]
- Attempting to launch using gunicorn. Not working. changed shell params
  to ros params and now using dynamic reconfigure. Still Buggy...
  [AlexV]
- Adding *geany files to ignore list. [AlexV]
- Subscribing and publishing to a topic from web page now woks. [AlexV]
- Adding missing accept header check to decide which format the response
  should be. [AlexV]
- Changing a lot of code, to migrate to flask. [AlexV]
- Beginning of a structure for a second WSGI app. Goal is to handle
  generation of web page as browser-based client for ROSTful. [AlexV]
- Improving README with Test steps for REST interface. [AlexV]
- Adding a launch file to test rostful with turtlesim. [AlexV]
- Improving README with steps to follow to test rostful. [AlexV]
- Adding werkzeug as dependency. changing repository and bug tracker
  URL. [AlexV]
- Adding static content, now using werkzeug. still some routing
  problems. [AlexV]
- Adding debug message. changing default port to 8080 to avoid
  permission problems. [AlexV]
- Add emailg. [Jihoon Lee]
- Add maintainter. [Jihoon Lee]
- Catkinize rostful package. [Jihoon Lee]
- First commit to convert to catkin. [AlexV]
- Create README.md. [Ben Kehoe]
- Rosdef on action submethods, renamed proxy to client. [Ben Kehoe]

  --HG--
  rename : scripts/proxy => scripts/client
  rename : src/rostful/proxy.py => src/rostful/client.py
- Full definitions & json format. [Ben Kehoe]
- Fixed connecting to individual service/topic/action. [Ben Kehoe]
- Added action capability. [Ben Kehoe]
- Updated deffile. [Ben Kehoe]
- Bug fixes. [Ben Kehoe]
- Fixed deffile, added prefix options to proxy. [Ben Kehoe]
- Updated deffile. [Ben Kehoe]
- Updated deffile with changes from raas. [Ben Kehoe]
- Binary working now. [Ben Kehoe]
- Interoperability with RaaS. [Ben Kehoe]
- Changed server to be WSGI-compatible. [Ben Kehoe]


