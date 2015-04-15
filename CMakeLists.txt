cmake_minimum_required(VERSION 2.8.3)
project(rostful)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package()

install(PROGRAMS
   scripts/client
   scripts/server
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
