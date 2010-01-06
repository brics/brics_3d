CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

PROJECT( flann )

# optimization
#IF(NOT WIN32)
#    ADD_DEFINITIONS(-O3 -Wall -finline-functions)
#    ADD_DEFINITIONS(-pipe -march=native)
#ENDIF(NOT WIN32)

## do compilations
ADD_SUBDIRECTORY(./src/cpp) # compile everything in src to create the libraries


