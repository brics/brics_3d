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
    ${Boost_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
   #${CMAKE_HOME_DIRECTORY}/external/IVT/lib/linux
   ${CMAKE_HOME_DIRECTORY}/external/6dslam/lib
   ${CGAL_LIBRARIES_DIR}
   ${CGAL_3RD_PARTY_LIBRARIES_DIRS}
)


# compile and link the executables 
ADD_EXECUTABLE(depth_image_stream_test depth_image_stream_test)
TARGET_LINK_LIBRARIES(depth_image_stream_test brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_EXECUTABLE(depth_image_transformation depth_image_transformation)
TARGET_LINK_LIBRARIES(depth_image_transformation brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

IF (USE_OSG)
    ADD_EXECUTABLE(osg_point_plotter osg_point_plotter)
    TARGET_LINK_LIBRARIES(osg_point_plotter 
        brics_3dpm_core 
        brics_3dpm_algorithm
        brics_3dpm_util
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
ENDIF(USE_OSG)

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

