# author: Sebastian Blumenthal
# date: 2009-09-09

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(BRICS_3D)

## check prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")

# search for OpenCV library
#INCLUDE(${PROJECT_SOURCE_DIR}/external/cmake_modules/FindOpenCV.cmake)

FIND_PACKAGE(OpenCV REQUIRED)
IF (OpenCV_FOUND)
  MESSAGE(STATUS "SUCCESSFUL: OpenCV found")
ELSE (OpenCV_FOUND)
  MESSAGE(STATUS "WARNING: OpenCV not found")
ENDIF (OpenCV_FOUND)

# search for Boost library (not yet necessary!)
FIND_PACKAGE(Boost)
IF (NOT Boost_INCLUDE_DIR)
  MESSAGE(STATUS "WARNING: Boost not found")
ELSE (NOT Boost_INCLUDE_DIR) 
  MESSAGE(STATUS "SUCCESSFUL: Boost found")
ENDIF()

# search for GGAL >= 3.4 library
FIND_PACKAGE(CGAL)
IF (NOT CGAL_FOUND)
  MESSAGE(STATUS "WARNING: CGAL not found")
ELSE (NOT CGAL_FOUND) 
  MESSAGE(STATUS "SUCCESSFUL: CGAL found")
ENDIF()
#MESSAGE(STATUS "INFO: "${CGAL_DIR} , ${CGAL_INCLUDE_DIRS})

IF (WIN32)
	SET (DL_LIB "")
ELSEIF (WIN32)
	SET (DL_LIB "dl")
ENDIF (WIN32)


## set parameters
# set parameters for compiler
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
#SET(CMAKE_VERBOSE_MAKEFILE ON) #shows compilation commands,etc.

# set parameters for build paths
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")
#SET(INCLUDE_INSTALL_DIR /usr/local/include/brics CACHE PATH "The subdirectory to the header prefix")
#SET(LIBRARY_INSTALL_DIR /usr/local/lib/brics CACHE PATH "The subdirectory to the header prefix")

## do compilations
ADD_SUBDIRECTORY(./src) #compile everything in src to create the libraries
#ADD_SUBDIRECTORY(./tests) #compile everything in tests to create the unit test binaries
ADD_SUBDIRECTORY(./examples) #compile everything in examples to create the example binaries

