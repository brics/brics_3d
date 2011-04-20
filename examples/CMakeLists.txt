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
    ${EIGEN2_INCLUDE_DIR}
    ${STANN_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
   #${CMAKE_HOME_DIRECTORY}/external/IVT/lib/linux
   ${CMAKE_HOME_DIRECTORY}/external/6dslam/lib
   ${CGAL_LIBRARIES_DIR}
   ${CGAL_3RD_PARTY_LIBRARIES_DIRS}
   ${OpenCV_LINK_DIRECTORIES}
)


# compile and link the executables 
ADD_EXECUTABLE(depth_image_stream_test depth_image_stream_test)
TARGET_LINK_LIBRARIES(depth_image_stream_test brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_EXECUTABLE(depth_image_transformation depth_image_transformation)
TARGET_LINK_LIBRARIES(depth_image_transformation brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

IF (USE_OSG)

    ADD_EXECUTABLE(BRICS_3D_Demo BRICS_3D_Demo)
    TARGET_LINK_LIBRARIES(BRICS_3D_Demo 
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )

    ADD_EXECUTABLE(osg_point_plotter osg_point_plotter)
    TARGET_LINK_LIBRARIES(osg_point_plotter 
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(osg_triangle_plotter osg_triangle_plotter)
    TARGET_LINK_LIBRARIES(osg_triangle_plotter 
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
 
    ADD_EXECUTABLE(pointcloud_viewer pointcloud_viewer)
    TARGET_LINK_LIBRARIES(pointcloud_viewer
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    	
    ADD_EXECUTABLE(fused_data_registration fused_data_registration)
    TARGET_LINK_LIBRARIES(fused_data_registration
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )    	
    	
    ADD_EXECUTABLE(osg_delaunay osg_delaunay)
    TARGET_LINK_LIBRARIES(osg_delaunay
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGUTIL_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}   	
    )
    
    ADD_EXECUTABLE(icp_test icp_test) 
    TARGET_LINK_LIBRARIES(icp_test
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(icp_benchmark icp_benchmark) 
    TARGET_LINK_LIBRARIES(icp_benchmark
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(point3D_benchmark point3D_benchmark) 
    TARGET_LINK_LIBRARIES(point3D_benchmark 
        brics_3dpm_core 
        brics_3dpm_algorithm 
        brics_3dpm_util
        ${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
ENDIF(USE_OSG)

#ADD_EXECUTABLE(flann_test flann_test)
#TARGET_LINK_LIBRARIES(flann_test brics_3dpm_core brics_3dpm_algorithm flann_s)

ADD_EXECUTABLE(rigidTransformation_benchmark rigidTransformation_benchmark)
TARGET_LINK_LIBRARIES(rigidTransformation_benchmark brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_EXECUTABLE(pointCorrespondence_benchmark pointCorrespondence_benchmark)
TARGET_LINK_LIBRARIES(pointCorrespondence_benchmark brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_EXECUTABLE(alpha_shapes alpha_shapes)
#TARGET_LINK_LIBRARIES(alpha_shapes CGAL)
TARGET_LINK_LIBRARIES(alpha_shapes ${CGAL_LIBRARY})

#ADD_EXECUTABLE(surface_mesher surface_mesher) # doesen't work so far
#TARGET_LINK_LIBRARIES(surface_mesher CGAL CGAL_ImageIO)

#ADD_EXECUTABLE(icp_test icp_test) #requires IVT to be installed
#TARGET_LINK_LIBRARIES(icp_test ivt QtCore brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)




ADD_DEFINITIONS(-DMAX_OPENMP_NUM_THREADS=4 -DOPENMP_NUM_THREADS=4)

IF(NOT WIN32)
  ADD_DEFINITIONS(-Wno-deprecated) # suppress warnings regarding beackward compability
ENDIF(NOT WIN32)

ADD_EXECUTABLE(slam6D slam6D)
TARGET_LINK_LIBRARIES(slam6D scanlib 6dslam_core newmat_cmake ${DL_LIB})

ADD_EXECUTABLE(slam6d_icp_test slam6d_icp_test)
TARGET_LINK_LIBRARIES(slam6d_icp_test scanlib  6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?

ADD_EXECUTABLE(icp_alpha_shapes_test icp_alpha_shapes_test)
#TARGET_LINK_LIBRARIES(icp_alpha_shapes_test CGAL scanlib  6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?
#TARGET_LINK_LIBRARIES(icp_alpha_shapes_test scanlib 6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?
TARGET_LINK_LIBRARIES(icp_alpha_shapes_test ${CGAL_LIBRARY} scanlib 6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?


ADD_EXECUTABLE(icp_alpha_shapes_stream_test icp_alpha_shapes_stream_test)
#TARGET_LINK_LIBRARIES(icp_alpha_shapes_stream_test CGAL scanlib  6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)
#TARGET_LINK_LIBRARIES(icp_alpha_shapes_stream_test scanlib 6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)
TARGET_LINK_LIBRARIES(icp_alpha_shapes_stream_test ${CGAL_LIBRARY} scanlib 6dslam_core newmat_cmake ${DL_LIB} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

#Executables for evaluation and comparison demo

ADD_EXECUTABLE(demoEvaluation demoEvaluation)
TARGET_LINK_LIBRARIES(demoEvaluation brics_3dpm_algorithm)

ADD_EXECUTABLE(demoComparison demoComparison)
TARGET_LINK_LIBRARIES(demoComparison brics_3dpm_algorithm)

#Executables for 3D segmentation demo
ADD_EXECUTABLE(demoSegmentationPlane demoSegmentationPlane)
TARGET_LINK_LIBRARIES(demoSegmentationPlane brics_3dpm_algorithm)
