# author: Sebastian Blumenthal
# date: 2009-09-09

SET(BRICS_3D_LIBRARIES_INCLUDE_DIRS 
	${CMAKE_HOME_DIRECTORY}/src
	${CMAKE_HOME_DIRECTORY}/external
    ${OPENCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${STANN_INCLUDE_DIRS}
)

IF (USE_EIGEN3)
	list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
ELSE (USE_EIGEN3)
	list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${EIGEN2_INCLUDE_DIR})
ENDIF (USE_EIGEN3)

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
    ${Boost_LIBRARIES}
)

SET(WORLD_MODEL_LIBRARY_LIBS

)

# define optional libraries
IF(XERCES_INCLUDE_DIR)
	list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${XERCES_INCLUDE_DIR})
	list(APPEND UTIL_LIBRARY_LIBS               ${XERCES_LIBRARIES})
ENDIF(XERCES_INCLUDE_DIR)	



# define sources lists for libraries (cpp files)
SET (CORE_LIBRARY_SOURCES
    ./core/IHomogeneousMatrix44
    ./core/HomogeneousMatrix44
	./core/PointCloud3D
	./core/PointCloud3DIterator
    ./core/Vector3D
    ./core/Normal3D
    ./core/NormalSet3D
	./core/Point3D
	./core/Point3DDecorator
	./core/ColoredPoint3D
	./core/Point3DIntensity
	./core/CorrespondenceIndexPair
	./core/CorrespondencePoint3DPair
	./core/Triangle
	./core/ITriangleMesh
	./core/TriangleMeshExplicit
	./core/TriangleMeshImplicit
    ./core/Logger
    ./core/ColorSpaceConvertor
    ./core/Version
    ./core/ParameterSet
)

SET (ALGORITHM_LIBRARY_SOURCES
	./algorithm/depthPerception/DepthImageToPointCloudTransformation
	
	./algorithm/featureExtraction/Centroid3D
    ./algorithm/featureExtraction/BoundingBox3DExtractor
	./algorithm/featureExtraction/INormalEstimation
	./algorithm/featureExtraction/PCA

    ./algorithm/filtering/IFiltering
	./algorithm/filtering/IOctreeReductionFilter
	./algorithm/filtering/IOctreePartition	
    ./algorithm/filtering/IOctreeSetup
	./algorithm/filtering/Octree
    ./algorithm/filtering/IColorBasedROIExtractor  
    ./algorithm/filtering/ColorBasedROIExtractorHSV
    ./algorithm/filtering/ColorBasedROIExtractorRGB    
    ./algorithm/filtering/BoxROIExtractor
    ./algorithm/filtering/MaskROIExtractor    


    ./algorithm/nearestNeighbor/INearestNeighbor
    ./algorithm/nearestNeighbor/INearestNeighborSetup
    ./algorithm/nearestNeighbor/INearestPoint3DNeighbor
    ./algorithm/nearestNeighbor/NearestNeighborANN
    ./algorithm/nearestNeighbor/NearestNeighborFLANN
    ./algorithm/nearestNeighbor/NearestNeighborSTANN
    
    .//algorithm/registration/IRegistration
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
	./algorithm/registration/IterativeClosestPointFactory
	
	./algorithm/meshGeneration/IMeshGeneration
	./algorithm/meshGeneration/IDelaunayTriangulation
		
    ./algorithm/segmentation/evaluation/Comparator
    ./algorithm/segmentation/evaluation/Evaluator
    ./algorithm/segmentation/evaluation/FileReader
    ./algorithm/segmentation/evaluation/FileWriter
    ./algorithm/segmentation/evaluation/MetricCalculator
    ./algorithm/segmentation/evaluation/gnuplot_i
    ./algorithm/segmentation/objectModels/IObjectModel
    ./algorithm/segmentation/objectModels/IObjectModelUsingNormals
    ./algorithm/segmentation/objectModels/ObjectModelPlane
    ./algorithm/segmentation/objectModels/ObjectModelPlaneFromLines
    ./algorithm/segmentation/objectModels/ObjectModelPlaneFromLineAndPoint
    ./algorithm/segmentation/objectModels/ObjectModelOrientedPlane
    ./algorithm/segmentation/objectModels/ObjectModelNormalPlane
    ./algorithm/segmentation/objectModels/ObjectModelCircle
    ./algorithm/segmentation/objectModels/ObjectModelSphere
    ./algorithm/segmentation/objectModels/ObjectModelLine
    ./algorithm/segmentation/objectModels/ObjectModelOrientedLine
    ./algorithm/segmentation/objectModels/ObjectModelCylinder	
	./algorithm/segmentation/SACMethods/ISACMethods	
	./algorithm/segmentation/SACMethods/SACMethodALMeDS
	./algorithm/segmentation/SACMethods/SACMethodRANSAC
	./algorithm/segmentation/SACMethods/SACMethodMSAC
	./algorithm/segmentation/SACMethods/SACMethodLMeDS
	./algorithm/segmentation/SACMethods/SACMethodMLESAC
    ./algorithm/segmentation/RegionBasedSACSegmentation.h
    ./algorithm/segmentation/RegionBasedSACSegmentationUsingNormals.h
    ./algorithm/segmentation/EuclideanClustering          
)

SET (UTIL_LIBRARY_SOURCES
	./util/DepthImageLoader
	./util/ConfigurationFileHandler
	./util/IpaDatasetLoader
	./util/Timer
	./util/Benchmark
	./util/SimplePointCloudGeneratorCube
)	

SET (WORLD_MODEL_LIBRARY_SOURCES
    ./worldModel/SceneObject
    ./worldModel/WorldModel
    
    ./worldModel/sceneGraph/
    ./worldModel/sceneGraph/Attribute
    ./worldModel/sceneGraph/Box
    ./worldModel/sceneGraph/Cylinder
    ./worldModel/sceneGraph/GeometricNode
    ./worldModel/sceneGraph/Group
    ./worldModel/sceneGraph/Mesh
    ./worldModel/sceneGraph/Node
    ./worldModel/sceneGraph/PointCloud
    ./worldModel/sceneGraph/SceneGraphFacade
    ./worldModel/sceneGraph/Shape
    ./worldModel/sceneGraph/SimpleIdGenerator
    ./worldModel/sceneGraph/TimeStamp
    ./worldModel/sceneGraph/Transform
    ./worldModel/sceneGraph/PathCollector
    ./worldModel/sceneGraph/AttributeFinder
    ./worldModel/sceneGraph/DotGraphGenerator    
    ./worldModel/sceneGraph/OutdatedDataDeleter
    ./worldModel/sceneGraph/OutdatedDataIdAwareDeleter
    ./worldModel/sceneGraph/PointCloudAccumulator
    ./worldModel/sceneGraph/PointCloudAccumulatorIdAware
    ./worldModel/sceneGraph/DotVisualizer
    ./worldModel/sceneGraph/SubGraphChecker
    ./worldModel/sceneGraph/SceneGraphToUpdatesTraverser     
)

#optional sources
IF (USE_OSG)
    list(APPEND UTIL_LIBRARY_SOURCES ./util/OSGPointCloudVisualizer)
    list(APPEND UTIL_LIBRARY_SOURCES ./util/OSGTriangleMeshVisualizer)
    list(APPEND ALGORITHM_LIBRARY_SOURCES ./algorithm/meshGeneration/DelaunayTriangulationOSG)
    list(APPEND WORLD_MODEL_LIBRARY_SOURCES ./worldModel/sceneGraph/OSGVisualizer)
    list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${OSG_INCLUDE_DIR})
ENDIF(USE_OSG)

IF (USE_PCL AND USE_EIGEN3)
    list(APPEND UTIL_LIBRARY_SOURCES ./util/PCLTypecaster)
    list(APPEND ALGORITHM_LIBRARY_SOURCES ./algorithm/segmentation/EuclideanClusteringPCL) 
    list(APPEND ALGORITHM_LIBRARY_SOURCES ./algorithm/segmentation/RGBColorBasedEuclideanClustering)
    list(APPEND BRICS_3D_LIBRARIES_INCLUDE_DIRS ${PCL_INCLUDE_DIRS} )
#    list(APPEND UTIL_LIBRARY_LIBS  pcl_io)
#    list(APPEND ALGORITHM_LIBRARY_LIBS  pcl_io pcl_segmentation)
#    list(APPEND BRICS_3D_LIBRARIES_LIB_DIRS ${PCL_LIBRARY_DIRS} )
    list(APPEND UTIL_LIBRARY_LIBS ${PCL_IO_LIBRARY})
    list(APPEND ALGORITHM_LIBRARY_LIBS ${PCL_IO_LIBRARY} ${PCL_SEGMENTATION_LIBRARY} )
    list(APPEND BRICS_3D_LIBRARIES_LIB_DIRS ${PCL_LIBRARY_DIRS})
ENDIF(USE_PCL AND USE_EIGEN3)    

# add library directories (-L)
LINK_DIRECTORIES(
    ${BRICS_3D_LIBRARIES_LIB_DIRS}
)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     ${BRICS_3D_LIBRARIES_INCLUDE_DIRS}
)


# compile libraries
ADD_LIBRARY(brics3d_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics3d_core ${CORE_LIBRARY_LIBS})

ADD_LIBRARY(brics3d_algorithm ${ALGORITHM_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics3d_algorithm brics3d_core ${ALGORITHM_LIBRARY_LIBS})

ADD_LIBRARY(brics3d_util ${UTIL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics3d_util brics3d_core ${UTIL_LIBRARY_LIBS})

ADD_LIBRARY(brics3d_world_model ${WORLD_MODEL_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(brics3d_world_model brics3d_core ${WORLD_MODEL_LIBRARY_LIBS})

INSTALL (
    TARGETS brics3d_core brics3d_algorithm brics3d_util brics3d_world_model
    RUNTIME DESTINATION bin/${BRICS_3D_INSTALL_NAME}
    LIBRARY DESTINATION lib/${BRICS_3D_INSTALL_NAME}
    ARCHIVE DESTINATION lib/${BRICS_3D_INSTALL_NAME}
)

INSTALL (DIRECTORY ./ 
    DESTINATION include/${BRICS_3D_INSTALL_NAME}
    FILES_MATCHING PATTERN "*.h"
    PATTERN .svn EXCLUDE
)

INSTALL (DIRECTORY ../external 
    DESTINATION include/${BRICS_3D_INSTALL_NAME}
    FILES_MATCHING PATTERN "*.h"
    PATTERN .svn EXCLUDE
)