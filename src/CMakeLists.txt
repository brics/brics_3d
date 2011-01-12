# author: Sebastian Blumenthal
# date: 2009-09-09

SET(BRICS_3D_LIBRARIES_INCLUDE_DIRS 
	${CMAKE_HOME_DIRECTORY}/src
	${CMAKE_HOME_DIRECTORY}/external
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN2_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${STANN_INCLUDE_DIRS}
)

SET(BRICS_3D_LIBRARIES_LIB_DIRS
    /usr/lib 
    ${OpenCV_LINK_DIRECTORIES}
)

# define required libraries
SET(CORE_LIBRARY_LIBS

)

SET(ALGORITHM_LIBRARY_LIBS
    scanlib 
    6dslam_core 
    newmat_cmake    
    ${DL_LIB}
    flann_s
    ANN
)

SET(UTIL_LIBRARY_LIBS
    ${OpenCV_LIBRARIES}
)


# define optionl libraries
IF(XERCES_INCLUDE_DIR)
	list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${XERCES_INCLUDE_DIR})
	list(APPEND UTIL_LIBRARY_LIBS               ${XERCES_LIBRARIES})
ENDIF(XERCES_INCLUDE_DIR)	



# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./core/IHomogeneousMatrix44
    ./core/HomogeneousMatrix44
	./core/PointCloud3D
	./core/ColoredPointCloud3D
	./core/Point3D
	./core/Point3DDecorator
	./core/ColoredPoint3D
	./core/CorrespondenceIndexPair
	./core/CorrespondencePoint3DPair
	./core/Triangle
	./core/ITriangleMesh
	./core/TriangleMeshExplicit
	./core/TriangleMeshImplicit
    ./core/Logger
)

SET (ALGORITHM_LIBRARY_SOURCES
	./algorithm/depthPerception/DepthImageToPointCloudTransformation
	./algorithm/filtering/IOctreeReductionFilter
	./algorithm/filtering/IOctreePartition	
    ./algorithm/filtering/IOctreeSetup
	./algorithm/filtering/Octree
    ./algorithm/nearestNeighbor/INearestNeighbor
    ./algorithm/nearestNeighbor/INearestNeighborSetup
    ./algorithm/nearestNeighbor/INearestPoint3DNeighbor
    ./algorithm/nearestNeighbor/NearestNeighborANN
    ./algorithm/nearestNeighbor/NearestNeighborFLANN
    ./algorithm/nearestNeighbor/NearestNeighborSTANN
	./algorithm/registration/IPointCorrespondence
	./algorithm/registration/PointCorrespondenceKDTree
	./algorithm/registration/PointCorrespondenceGenericNN
	./algorithm/registration/IRigidTransformationEstimation
	./algorithm/registration/RigidTransformationEstimationSVD
	./algorithm/registration/RigidTransformationEstimationQUAT
	./algorithm/registration/RigidTransformationEstimationHELIX
	./algorithm/registration/RigidTransformationEstimationAPX
	./algorithm/registration/RigidTransformationEstimationORTHO
	./algorithm/registration/IIterativeClosestPoint
	./algorithm/registration/IIterativeClosestPointDetailed
	./algorithm/registration/IterativeClosestPoint
	./algorithm/registration/IterativeClosestPoint6DSLAM
	./algorithm/registration/IterativeClosestPointFactory
	./algorithm/meshGeneration/IDelaunayTriangulation
	
	

)

SET (UTIL_LIBRARY_SOURCES
	./util/DepthImageLoader
	./util/ConfigurationFileHandler
	./util/IpaDatasetLoader
	./util/Timer
	./util/Benchmark
)	

#optional sources
IF (USE_OSG)
    list(APPEND UTIL_LIBRARY_SOURCES ./util/OSGPointCloudVisualizer)
    list(APPEND UTIL_LIBRARY_SOURCES ./util/OSGTriangleMeshVisualizer)
    list(APPEND ALGORITHM_LIBRARY_SOURCES ./algorithm/meshGeneration/DelaunayTriangulationOSG)
    list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${OSG_INCLUDE_DIR})
ENDIF(USE_OSG)


# add library directories (-L)
LINK_DIRECTORIES(
    ${BRICS_3D_LIBRARIES_LIB_DIRS}
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     ${BRICS_3D_LIBRARIES_INCLUDE_DIRS}
)


# compile libraries
ADD_LIBRARY(brics_3dpm_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_core ${CORE_LIBRARY_LIBS})

ADD_LIBRARY(brics_3dpm_algorithm ${ALGORITHM_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_algorithm brics_3dpm_core ${ALGORITHM_LIBRARY_LIBS})

ADD_LIBRARY(brics_3dpm_util ${UTIL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics_3dpm_util brics_3dpm_core ${UTIL_LIBRARY_LIBS})

