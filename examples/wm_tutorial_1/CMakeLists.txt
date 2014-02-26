CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(wm_tutorial_1)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
#FIND_PACKAGE(BRICS_3D REQUIRED) # Already included from top level CMakeFiles

# add include directories (-I)
INCLUDE_DIRECTORIES(${BRICS_3D_INCLUDE_DIRS})

# add library directories (-L)
LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES})

# compile and link the executables
ADD_EXECUTABLE(wm_tutorial_1 wm_tutorial_1.cpp) 
TARGET_LINK_LIBRARIES(wm_tutorial_1 ${BRICS_3D_LIBRARIES})


