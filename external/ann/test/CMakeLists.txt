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
SET(ANN_TEST_SOURCES
	ann_test
	rand
)


# compile binary
ADD_EXECUTABLE(ann_test ${ANN_TEST_SOURCES})
TARGET_LINK_LIBRARIES(ann_test ANN) 

