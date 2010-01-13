# author: Sebastian Blumenthal
# date: 2010-01-12

# add include directories (-I)
INCLUDE_DIRECTORIES(
	. 
#	${CMAKE_HOME_DIRECTORY}/include/
	${CMAKE_HOME_DIRECTORY}/external
)


# add library directories (-L)
LINK_DIRECTORIES(
	
)


# define sources lists for libraries (cpp/cc files)
SET(ANN_LIBRARY_SOURCES
	ANN
	brute
	kd_tree
	kd_util
	kd_split
	kd_dump
	kd_search
	kd_pr_search
	kd_fix_rad_search
	bd_tree
	bd_search
	bd_pr_search
	bd_fix_rad_search
	perf
)


# compile library
ADD_LIBRARY(ANN ${ANN_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(ANN m) #TODO check if necessary and test it it works for windows








