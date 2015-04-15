cmake_minimum_required(VERSION 2.8.3)
project(rostful)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint rospy actionlib_msgs)
catkin_python_setup()
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

## Unit tests
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
