# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories
INCLUDE_DIRECTORIES(
    . 
    ${CMAKE_HOME_DIRECTORY}/src 
    ${OPENCV_INCLUDE_DIR}
    ${CMAKE_HOME_DIRECTORY}/external
    ${CGAL_INCLUDE_DIRS}
    ${CGAL_3RD_PARTY_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${STANN_INCLUDE_DIRS}
)

IF (USE_EIGEN3)
	INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
ELSE (USE_EIGEN3)
	INCLUDE_DIRECTORIES(${EIGEN2_INCLUDE_DIR})
ENDIF (USE_EIGEN3)

LINK_DIRECTORIES(
   #${CMAKE_HOME_DIRECTORY}/external/IVT/lib/linux
   ${CMAKE_HOME_DIRECTORY}/external/6dslam/lib
   ${CGAL_LIBRARIES_DIR}
   ${CGAL_3RD_PARTY_LIBRARIES_DIRS}
   ${OpenCV_LINK_DIRECTORIES}
)

# compile and link the executables 
ADD_EXECUTABLE(depth_image_stream_test depth_image_stream_test)
TARGET_LINK_LIBRARIES(depth_image_stream_test brics3d_core brics3d_algorithm brics3d_util)

ADD_EXECUTABLE(depth_image_transformation depth_image_transformation)
TARGET_LINK_LIBRARIES(depth_image_transformation brics3d_core brics3d_algorithm brics3d_util)

IF (USE_OSG)

    ADD_EXECUTABLE(BRICS_3D_Demo BRICS_3D_Demo)
    TARGET_LINK_LIBRARIES(BRICS_3D_Demo 
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )

    ADD_EXECUTABLE(osg_point_plotter osg_point_plotter)
    TARGET_LINK_LIBRARIES(osg_point_plotter 
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(osg_triangle_plotter osg_triangle_plotter)
    TARGET_LINK_LIBRARIES(osg_triangle_plotter 
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
 
    ADD_EXECUTABLE(pointcloud_viewer pointcloud_viewer)
    TARGET_LINK_LIBRARIES(pointcloud_viewer
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    	  	    	
    ADD_EXECUTABLE(osg_delaunay osg_delaunay)
    TARGET_LINK_LIBRARIES(osg_delaunay
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGUTIL_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}   	
    )
    
    ADD_EXECUTABLE(icp_test icp_test) 
    TARGET_LINK_LIBRARIES(icp_test
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(icp_benchmark icp_benchmark) 
    TARGET_LINK_LIBRARIES(icp_benchmark
        brics3d_core 
        brics3d_algorithm
        brics3d_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(point3D_benchmark point3D_benchmark) 
    TARGET_LINK_LIBRARIES(point3D_benchmark 
        brics3d_core 
        brics3d_algorithm 
        brics3d_util
        ${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )	
     
    IF (USE_PCL AND USE_EIGEN3)	
        INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})

        ADD_EXECUTABLE(kinect_test kinect_test) 
        TARGET_LINK_LIBRARIES(kinect_test 
            brics3d_core 
            brics3d_algorithm 
            brics3d_util
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
        
        ADD_EXECUTABLE(extract_object_clusters extract_object_clusters) 
        TARGET_LINK_LIBRARIES(extract_object_clusters 
            brics3d_core 
            brics3d_algorithm 
            brics3d_util
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
        
        
        INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})    
        ADD_EXECUTABLE(detection_test detection_test) 
        TARGET_LINK_LIBRARIES(detection_test 
            brics3d_core 
            brics3d_algorithm 
            brics3d_util
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        	${PCL_KDTREE_LIBRARY}
        	${PCL_SEGMENTATION_LIBRARY} 
#        	${PCL_SAMPLE_CONSENSUS_LIBRARY} 
       	    ${PCL_FILTERS_LIBRARY} 
        	${PCL_IO_LIBRARY} 
        	${PCL_FEATURES_LIBRARY} 
        )
    ENDIF(USE_PCL AND USE_EIGEN3)    
    
ENDIF(USE_OSG)

ADD_EXECUTABLE(rigidTransformation_benchmark rigidTransformation_benchmark)
TARGET_LINK_LIBRARIES(rigidTransformation_benchmark brics3d_core brics3d_algorithm brics3d_util)

ADD_EXECUTABLE(pointCorrespondence_benchmark pointCorrespondence_benchmark)
TARGET_LINK_LIBRARIES(pointCorrespondence_benchmark brics3d_core brics3d_algorithm brics3d_util)


#ADD_DEFINITIONS(-DMAX_OPENMP_NUM_THREADS=4 -DOPENMP_NUM_THREADS=4)

IF(NOT WIN32)
  ADD_DEFINITIONS(-Wno-deprecated) # suppress warnings regarding beackward compability
ENDIF(NOT WIN32)

#Executables for evaluation and comparison demo
ADD_EXECUTABLE(demoEvaluation demoEvaluation)
TARGET_LINK_LIBRARIES(demoEvaluation brics3d_algorithm)

ADD_EXECUTABLE(demoComparison demoComparison)
TARGET_LINK_LIBRARIES(demoComparison brics3d_algorithm)

#Executables for 3D segmentation demo
ADD_EXECUTABLE(demoSegmentationPlane demoSegmentationPlane)
TARGET_LINK_LIBRARIES(demoSegmentationPlane brics3d_algorithm)

ADD_EXECUTABLE(demoSegmentationNormalPlane demoSegmentationNormalPlane)
TARGET_LINK_LIBRARIES(demoSegmentationNormalPlane brics3d_algorithm)

