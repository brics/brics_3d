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
    ./core/IHomogeneousMatrix44
    ./core/HomogeneousMatrix44
	./core/PointCloud3D
	./core/Point3D
)

SET (ALGORITHM_LIBRARY_SOURCES
	./algorithm/DepthImageToPointCloudTransformation
)

SET (UTIL_LIBRARY_SOURCES
	./util/DepthImageLoader
)	


# compile libraries
ADD_LIBRARY(brics_3dpm_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_core ${OPENCV_LIBS})

ADD_LIBRARY(brics_3dpm_algorithm ${ALGORITHM_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_algorithm brics_3dpm_core ${OPENCV_LIBS})

ADD_LIBRARY(brics_3dpm_util ${UTIL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_util brics_3dpm_core ${OPENCV_LIBS})

