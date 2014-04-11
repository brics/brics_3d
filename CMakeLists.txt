# author: Sebastian Blumenthal
# date: 2009-09-09

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(BRICS_3D)

## project definitions
ADD_DEFINITIONS(-DBRICS_IMGAGES_DIR="${CMAKE_HOME_DIRECTORY}/examples/test_data/depth_images")
ADD_DEFINITIONS(-DBRICS_MODELS_DIR="${CMAKE_HOME_DIRECTORY}/examples/test_data/3d_models")
ADD_DEFINITIONS(-DBRICS_CONFIGURATIONS_DIR="${CMAKE_HOME_DIRECTORY}/examples/test_data")
ADD_DEFINITIONS(-DBRICS_LOGFILES_DIR="${CMAKE_HOME_DIRECTORY}/examples/test_data/logfiles")
SET(CMAKE_CXX_FLAGS "-fPIC")

##  Loads the current version number:
FILE(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" CMAKE_BRICS_3D_VERSION_NUMBER)

# For example: "0.1.0"
STRING(REGEX MATCHALL "[0-9]+" CMAKE_BRICS_3D_VERSION_PARTS "${CMAKE_BRICS_3D_VERSION_NUMBER}")

LIST(GET CMAKE_BRICS_3D_VERSION_PARTS 0 CMAKE_BRICS_3D_VERSION_NUMBER_MAJOR)
LIST(GET CMAKE_BRICS_3D_VERSION_PARTS 1 CMAKE_BRICS_3D_VERSION_NUMBER_MINOR)
LIST(GET CMAKE_BRICS_3D_VERSION_PARTS 2 CMAKE_BRICS_3D_VERSION_NUMBER_PATCH)
SET(CMAKE_BRICS_3D_VERSION "BRICS_3D ${CMAKE_BRICS_3D_VERSION_NUMBER_MAJOR}.${CMAKE_BRICS_3D_VERSION_NUMBER_MINOR}.${CMAKE_BRICS_3D_VERSION_NUMBER_PATCH}")
SET(CMAKE_BRICS_3D_VERSION_CODE "0x${CMAKE_BRICS_3D_VERSION_NUMBER_MAJOR}${CMAKE_BRICS_3D_VERSION_NUMBER_MINOR}${CMAKE_BRICS_3D_VERSION_NUMBER_PATCH}")
ADD_DEFINITIONS(-DBRICS_3D_VERSION=${CMAKE_BRICS_3D_VERSION})
ADD_DEFINITIONS(-DBRICS_3D_VERSION_CODE=${CMAKE_BRICS_3D_VERSION_CODE})

## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")

MESSAGE("Checking required prerequisites: ")



# search for Boost library (not yet necessary!)
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0"  "1.42" "1.42.0")
FIND_PACKAGE(Boost COMPONENTS thread)
IF (NOT Boost_INCLUDE_DIR)
  MESSAGE(SEND_ERROR "WARNING: Boost not found")
ELSE (NOT Boost_INCLUDE_DIR) 
  MESSAGE(STATUS "SUCCESSFUL: Boost found")
ENDIF()

## search for GGAL >= 3.4 library
#OPTION(USE_CGAL "Enable CGAL" OFF)
#IF (USE_CGAL)
#    FIND_PACKAGE(CGAL)
#    IF (NOT CGAL_FOUND)
#      MESSAGE(SEND_ERROR "WARNING: CGAL not found")
#    ELSE (NOT CGAL_FOUND) 
#      MESSAGE(STATUS "SUCCESSFUL: CGAL found")
#    ENDIF()
#    #MESSAGE(STATUS "INFO: "${CGAL_DIR} , ${CGAL_INCLUDE_DIRS})
#    #MESSAGE("CGAL lib: "${CGAL_LIBRARY})
#ENDIF (USE_CGAL)    


OPTION(USE_EIGEN3 "Enable Eigen version 3 " ON)
IF (USE_EIGEN3)
    # Eigen 3
    find_package(Eigen REQUIRED)
    ADD_DEFINITIONS(-DEIGEN3)
ELSE (USE_EIGEN3)     
    #search for Eigen2 (header-only) library
    FIND_PACKAGE(Eigen2)
    IF (EIGEN2_FOUND)
        MESSAGE(STATUS "SUCCESSFUL: Eigen2 found")
    ELSE (EIGEN2_FOUND)
        MESSAGE(SEND_ERROR "WARNING: Eigen2 not found")
    ENDIF (EIGEN2_FOUND)
ENDIF (USE_EIGEN3)



# set STANN path (as they only use <> in headers ...) 
SET(STANN_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/external/stann/include)

## check optional prerequisites
MESSAGE("Checking optional prerequisites: ")

# search for cpp unit testing library
OPTION(USE_CPPUNIT "Enable cppunit test." OFF)
IF(USE_CPPUNIT)
    FIND_PACKAGE(CppUnit)
    IF (CppUnit_FOUND)
      MESSAGE(STATUS "SUCCESSFUL: CppUnit found")
    ELSE (CppUnit_FOUND)
      MESSAGE(STATUS "WARNING: CppUnit not found")
    ENDIF (CppUnit_FOUND)
ENDIF(USE_CPPUNIT)

# search for open scene graph (OSG) visualization library 
FIND_PACKAGE(OpenSceneGraph COMPONENTS osgGA osgUtil osgViewer osgText)
IF (OPENSCENEGRAPH_FOUND)
	message(STATUS "SUCCESSFUL: OpenSceneGraph found")
	ADD_DEFINITIONS(-DBRICS_OSG_ENABLE)	
ELSE (OPENSCENEGRAPH_FOUND)
	MESSAGE(STATUS "WARNING: OpenSceneGraph not found")
	SET(OPENSCENEGRAPH_INCLUDE_DIRS "")
	SET(OSG_LIBRARY_DEBUG "")
	SET(OSGGA_LIBRARY_DEBUG "")
	SET(OSGVIEWER_LIBRARY_DEBUG "")
	SET(REMOVE_DEFINITIONS(-DBRICS_OSG_ENABLE))
ENDIF (OPENSCENEGRAPH_FOUND)
OPTION(USE_OSG "Enable Open Scene Graph" OFF)

# search for OpenCV library
OPTION(USE_OPENCV "Enable OpenCV Support" OFF)
IF(USE_OPENCV)  
    FIND_PACKAGE(OpenCV REQUIRED)
    IF (OpenCV_FOUND)
      MESSAGE(STATUS "SUCCESSFUL: OpenCV found")
      ADD_DEFINITIONS(-DBRICS_OPENCV_ENABLE)
    ELSE (OpenCV_FOUND)
      MESSAGE(SEND_ERROR "WARNING: OpenCV not found")
    ENDIF (OpenCV_FOUND)
ENDIF(USE_OPENCV)

# search for PCL and related libraries
OPTION(USE_PCL "Enable PCL Support" OFF)
IF (USE_PCL)  
    #PCL
    FIND_PACKAGE(PCL 1.0 REQUIRED COMPONENTS)
 #   include_directories(${PCL_INCLUDE_DIRS})

    #OpenNI
    FIND_PACKAGE(OpenNI)
    IF(OPENNI_FOUND)
#    include_directories(${OPENNI_INCLUDE_DIRS})
    message(STATUS "OpenNI found.")
    #message(STATUS "OpenNI include ${OPENNI_INCLUDE_DIR}")
    #message(STATUS "OpenNI libs ${OPENNI_LIBRARY}")
endif(OPENNI_FOUND)
ENDIF(USE_PCL)


# search for Xerces xml library
OPTION(USE_XERCES "Enable Xerces xml support for cinfig files" OFF)
IF (USE_XERCES)
    FIND_PACKAGE(Xerces)
    IF (XERCES_INCLUDE_DIR)
    	MESSAGE(STATUS "SUCCESSFUL: Xerces found")
    	SET(BRICS_XERCES_ENABLE 1)
    	ADD_DEFINITIONS(-DBRICS_XERCES_ENABLE)
    ELSE (XERCES_INCLUDE_DIR)
    	MESSAGE(STATUS "WARNING: Xerces not found")
    ENDIF (XERCES_INCLUDE_DIR)
ENDIF(USE_XERCES)


# search for Microblx
OPTION(USE_MICROBLX "Enable Microblx Support" OFF)
IF(USE_MICROBLX)  
    FIND_PACKAGE(Microblx REQUIRED)
    IF (MICROBLX_FOUND)
      MESSAGE(STATUS "SUCCESSFUL: MICROBLX found")
      ADD_DEFINITIONS(-DBRICS_MICROBLX_ENABLE)     
      GET_FILENAME_COMPONENT(MICROBLX_ROOT_DIR ${MICROBLX_INCLUDE_DIR} PATH)
      ADD_DEFINITIONS(-DMICROBLX_ROOT_DIR="${MICROBLX_ROOT_DIR}/") # We need to tell where e.g. the "standard types" are
    ELSE (MICROBLX_FOUND)
      MESSAGE(SEND_ERROR "WARNING: MICROBLX not found. You might need to set the MICROBLX_DIR environment variable.")
    ENDIF (MICROBLX_FOUND)
ENDIF(USE_MICROBLX)

# search for Microblx
OPTION(USE_HDF5 "Enable HDF5 Support" OFF)
IF(USE_HDF5)  
    FIND_PACKAGE(HDF5 REQUIRED COMPONENTS CXX)
    IF (HDF5_FOUND)
      MESSAGE(STATUS "SUCCESSFUL: HDF5 found")
      ADD_DEFINITIONS(-DBRICS_HDF5_ENABLE)     
    ELSE (HDF5_FOUND)
      MESSAGE(SEND_ERROR "WARNING: HDF5 not found.")
    ENDIF (HDF5_FOUND)
ENDIF(USE_HDF5)

## perform platform dependant configurations
IF (WIN32)   # windows
	SET (CGAL_LIBRARY "") # windows uses auto_linker.h => skip CGAL_LIBRARY for linker
	SET (DL_LIB "")		
ELSE (WIN32) # linux
	SET (DL_LIB "dl")
	ADD_DEFINITIONS(-Wall)	
ENDIF (WIN32)


## set parameters
# set parameters for compiler
SET(CMAKE_BUILD_TYPE DEBUG) #enable debug mode (e.g. for embedded gdb in eclipse )
#SET(CMAKE_VERBOSE_MAKEFILE ON) #shows compilation commands,etc.

include(UseCodeCoverage) # for codecoverage tools with gcc (optional)

# set parameters for build paths
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")
SET(BRICS_3D_INSTALL_NAME brics_3d)
#SET(INCLUDE_INSTALL_DIR /usr/local/include/brics CACHE PATH "The subdirectory to the header prefix")
#SET(LIBRARY_INSTALL_DIR /usr/local/lib/brics CACHE PATH "The subdirectory to the header prefix")

## do compilations
ADD_SUBDIRECTORY(./external/6dslam)
ADD_SUBDIRECTORY(./external/flann)
ADD_SUBDIRECTORY(./external/ann)
ADD_SUBDIRECTORY(./src/brics_3d) #compile everything in src to create the libraries
ADD_SUBDIRECTORY(./examples) #compile everything in examples to create the example binaries

IF(CppUnit_FOUND)
    ADD_SUBDIRECTORY(./tests) #compile everything in tests to create the unit test binaries
ENDIF(CppUnit_FOUND)
    


