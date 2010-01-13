# author: Sebastian Blumenthal
# date: 2010-01-12

# add include directories (-I)
INCLUDE_DIRECTORIES(
	${CMAKE_HOME_DIRECTORY}/include/
)


# add library directories (-L)
LINK_DIRECTORIES(
	${CMAKE_HOME_DIRECTORY}/lib/
)


# define sources lists for libraries (cpp/cc files)
SET(ANN_ANN2FIG_SOURCES
	ann2fig
)


# compile binary
ADD_EXECUTABLE(ann2fig ${ANN_ANN2FIG_SOURCES})
TARGET_LINK_LIBRARIES(ann2fig ANN) 

