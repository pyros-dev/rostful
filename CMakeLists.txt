cmake_minimum_required(VERSION 2.8.3)
project(rostful)

find_package(catkin REQUIRED COMPONENTS
    catkin_pip
)

# We need to install the pip dependencies in the workspace being created
# catkin_pip_requirements(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)
# Currently : no extra dev requirements needed : every required dependency in setup.py

# Forcing to not retrieve all our dependencies, in order to uses system/ROS packages.
# TMP : working with pip dependencies
#set (CATKIN_PIP_NO_DEPS False FORCE)

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_target(rostful)

catkin_package()

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Unit tests
if (CATKIN_ENABLE_TESTING)

    ##############################################################################
    # Nose Tests
    ##############################################################################

    catkin_add_pytests(rostful/tests)

endif()
