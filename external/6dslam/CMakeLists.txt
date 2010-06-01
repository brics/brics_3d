# author: Sebastian Blumenthal
# date: 2009-09-09

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(6D_SLAM)


## set definitions (derived from Makefile.options)
ADD_DEFINITIONS(-DMAX_OPENMP_NUM_THREADS=4 -DOPENMP_NUM_THREADS=4)

# for linux
if ( CMAKE_COMPILER_IS_GNUCXX )
    ADD_DEFINITIONS(-Wno-deprecated)
    # optimization
    ADD_DEFINITIONS(-O3 -Wall -finline-functions)
    ADD_DEFINITIONS(-pipe -march=native)
endif ( CMAKE_COMPILER_IS_GNUCXX )

#for show binary:
# -lglut -lGL -lGLU -L/usr/X11R6/lib -lXmu -lXt -lXi -lm -lXext -lX11 -lpthread -ldl -lstdc++ 
#SET (GUI_LIBRARY_LIB_INCLUDES
#	/usr/X11R6/lib	
#)
#SET (GUI_LIBRARY_LIBS
#	glut
#	GL 
#	GLU 
#	Xmu 
#	Xt 
#	Xi 
#	m 
#	Xext 
#	X11 
#	pthread 
#	dl 
#	stdc++ 
#)

# for windows: TODO


## set parameters
# set parameters for compiler
SET(CMAKE_BUILD_TYPE DEBUG) # enable debug mode (e.g. for embedded gdb in eclipse)
#SET(CMAKE_VERBOSE_MAKEFILE ON) # shows compilation commands, etc.


# set parameters for build paths
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/bin/ CACHE PATH "Configure the executable output path.")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib/ CACHE PATH "Configure the library output path.")


## do compilations
ADD_SUBDIRECTORY(./src) # compile everything in src to create the libraries


