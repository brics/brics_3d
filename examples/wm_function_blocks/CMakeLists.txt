CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(testblock)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
#FIND_PACKAGE(BRICS_3D REQUIRED) # Already included from top level CMakeFiles

# add include directories (-I)
INCLUDE_DIRECTORIES(${BRICS_3D_INCLUDE_DIRS})

# add library directories (-L)
LINK_DIRECTORIES(${BRICS_3D_LINK_DIRECTORIES})

# compile and link the executables
ADD_LIBRARY(testblock SHARED testblock.cpp) 
TARGET_LINK_LIBRARIES(testblock 
        ${BRICS_3D_LIBRARIES}     	
)

ADD_EXECUTABLE(loader loader.cpp) 
TARGET_LINK_LIBRARIES(loader 
        ${BRICS_3D_LIBRARIES}     	
)
