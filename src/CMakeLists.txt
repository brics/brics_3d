# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories (-I)
INCLUDE_DIRECTORIES(
    . 
    ${CMAKE_HOME_DIRECTORY}/src
    ${CMAKE_HOME_DIRECTORY}/external 
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN2_INCLUDE_DIR}
)


# add library directories (-L)
LINK_DIRECTORIES(
    #/usr/local/lib 
    /usr/lib 
    ${OpenCV_LINK_DIRECTORIES}
)


# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./core/IHomogeneousMatrix44
    ./core/HomogeneousMatrix44
	./core/PointCloud3D
	./core/Point3D
	./core/CorrespondenceIndexPair
	./core/CorrespondencePoint3DPair
)

SET (ALGORITHM_LIBRARY_SOURCES
	./algorithm/DepthImageToPointCloudTransformation
#	./algorithm/filter/IOctree
	./algorithm/registration/IPointCorrespondence
	./algorithm/registration/PointCorrespondenceKDTree
	./algorithm/registration/IRigidTransformationEstimation
	./algorithm/registration/RigidTransformationEstimationSVD
	./algorithm/registration/RigidTransformationEstimationQUAT
	./algorithm/registration/RigidTransformationEstimationHELIX
	./algorithm/registration/RigidTransformationEstimationAPX
	./algorithm/registration/RigidTransformationEstimationORTHO
	./algorithm/registration/IIterativeClosestPoint
	./algorithm/registration/IterativeClosestPoint
	./algorithm/registration/IterativeClosestPoint6DSLAM

)

SET (UTIL_LIBRARY_SOURCES
	./util/DepthImageLoader
)	

#optional sources
IF (USE_OSG)
    SET (UTIL_LIBRARY_SOURCES
    ${UTIL_LIBRARY_SOURCES}
    ./util/OSGPointCloudVisualizer 
    )
ENDIF(USE_OSG)


# compile libraries
ADD_LIBRARY(brics_3dpm_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_core ${OpenCV_LIBRARIES})

ADD_LIBRARY(brics_3dpm_algorithm ${ALGORITHM_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_algorithm brics_3dpm_core ${OpenCV_LIBRARIES} scanlib 6dslam_core newmat_cmake ${DL_LIB})

ADD_LIBRARY(brics_3dpm_util ${UTIL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_util brics_3dpm_core ${OpenCV_LIBRARIES})

