# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories (-I)
INCLUDE_DIRECTORIES(
    . 
    ${CMAKE_HOME_DIRECTORY}/src 
    ${OPENCV_INCLUDE_DIRS}
)


# add library directories (-L)
LINK_DIRECTORIES(
    #/usr/local/lib 
    /usr/lib 
    ${OPENCV_LINK_DIRS}
)


# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
	./core/PointCloud3D
	./core/Point3D
)

SET (ALGORITHM_LIBRARY_SOURCES
	./algorithm/DepthImageToPointCloudTransformation
)

SET (UTIL_LIBRARY_SOURCES
	./util/DepthImageLoader
)	

# automatically search for cppUnit tests in "src/tests" folder
# NOTE: all test must end with Test.cpp e.g. Point3DTest.cpp
FILE(GLOB CPPUNITTEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "./tests/*Test.cpp" )
MESSAGE("CppUnitTests: ${CPPUNITTEST_SOURCES}")


# compile libraries
ADD_LIBRARY(brics_3dpm_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_core ${OPENCV_LIBS})

ADD_LIBRARY(brics_3dpm_algorithm ${ALGORITHM_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_algorithm brics_3dpm_core ${OPENCV_LIBS})

ADD_LIBRARY(brics_3dpm_util ${UTIL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_util brics_3dpm_core ${OPENCV_LIBS})


# compile unit testing executable 
ADD_EXECUTABLE(run_unit_tests tests/TestRunner.cpp ${CPPUNITTEST_SOURCES})
TARGET_LINK_LIBRARIES(run_unit_tests cppunit brics_3dpm_core)
