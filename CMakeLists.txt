cmake_minimum_required(VERSION 2.8.3)
project(rostful)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint rospy actionlib_msgs dynamic_reconfigure)
catkin_python_setup()

# make sure configure headers are built before any node using them
#add_dependencies(rostful ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_package()

# Install executable scripts
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} src/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS}
               ${${PROJECT_NAME}_PY_SRCS})

install(DIRECTORY migrations/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/migrations
)

install(DIRECTORY interactions/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/interactions
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Unit tests
if (CATKIN_ENABLE_TESTING)

# Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)
#TODO add_rostest(rostful.test)


endif()
