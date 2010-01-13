# author: Sebastian Blumenthal
# date: 2010-01-12

## general CMake definitions
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ANN)

## optimization
ADD_DEFINITIONS(-O3 -Wall)


## set parameters
# set parameters for compiler
SET(CMAKE_BUILD_TYPE DEBUG) # enable debug mode (e.g. for embedded gdb in eclipse)
#SET(CMAKE_VERBOSE_MAKEFILE ON) # shows compilation commands, etc.


# set parameters for build paths
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")


## do compilations
ADD_SUBDIRECTORY(./src) # compile everything in src to create the librariy
#ADD_SUBDIRECTORY(./sample)
#ADD_SUBDIRECTORY(./test)
#ADD_SUBDIRECTORY(./ann2fig)

