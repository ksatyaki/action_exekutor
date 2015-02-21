cmake_minimum_required(VERSION 2.8.3)
project(action_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES action_exekutor
  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library

######################################
######################################

add_library(action_exekutor
  src/action_exekutor.cpp
)

target_link_libraries(action_exekutor
  ${catkin_LIBRARIES} 
  peiskernel_mt
  peiskernel
  pthread
)

######################################
