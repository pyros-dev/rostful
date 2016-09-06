cmake_minimum_required(VERSION 2.8.3)
project(rostful)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS roslint rospy)
catkin_python_setup()

catkin_package()

# Install executable scripts
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

    catkin_add_nosetests(rostful/tests)

endif()
