cmake_minimum_required(VERSION 2.8.3)
project(rostful)

# Minimal Pure Python module setup
find_package(catkin REQUIRED COMPONENTS
    catkin_pure_python
    roslint
    rospy
)

# We need to install the pip dependencies in the workspace being created
catkin_pip_requirements(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)

# defining current package as a package that should be managed by pip (not catkin - even though we make it usable with workspaces)
catkin_pip_package()


## Unit tests
if (CATKIN_ENABLE_TESTING)

    ##############################################################################
    # Nose Tests
    ##############################################################################

    catkin_add_nosetests(rostful/tests)

endif()
