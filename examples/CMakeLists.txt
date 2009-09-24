# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories
INCLUDE_DIRECTORIES(
    . 
    ${CMAKE_HOME_DIRECTORY}/src 
    ${OPENCV_INCLUDE_DIRS}
    #${CMAKE_HOME_DIRECTORY}/external/IVT/src/
    ${CMAKE_HOME_DIRECTORY}/external/6dslam/src
)

LINK_DIRECTORIES(
   #${CMAKE_HOME_DIRECTORY}/external/IVT/lib/linux
   ${CMAKE_HOME_DIRECTORY}/external/6dslam/lib
)


# compile and link the executables 
ADD_EXECUTABLE(depth_image_stream_test depth_image_stream_test)
TARGET_LINK_LIBRARIES(depth_image_stream_test brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_EXECUTABLE(depth_image_transformation depth_image_transformation)
TARGET_LINK_LIBRARIES(depth_image_transformation brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)


ADD_EXECUTABLE(alpha_shapes alpha_shapes)
TARGET_LINK_LIBRARIES(alpha_shapes CGAL)

#ADD_EXECUTABLE(surface_mesher surface_mesher) # doesen't work so far
#TARGET_LINK_LIBRARIES(surface_mesher CGAL CGAL_ImageIO)

#ADD_EXECUTABLE(icp_test icp_test) #requires IVT to be installed
#TARGET_LINK_LIBRARIES(icp_test ivt QtCore brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)

ADD_DEFINITIONS(-DMAX_OPENMP_NUM_THREADS=4 -DOPENMP_NUM_THREADS=4)
ADD_DEFINITIONS(-Wno-deprecated) # suppress warnings regarding beackward compability
ADD_EXECUTABLE(slam6D slam6D)
TARGET_LINK_LIBRARIES(slam6D scanlib 6dslam_core newmat_cmake dl)

ADD_EXECUTABLE(slam6d_icp_test slam6d_icp_test)
TARGET_LINK_LIBRARIES(slam6d_icp_test scanlib  6dslam_core newmat_cmake dl brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?

ADD_EXECUTABLE(icp_alpha_shapes_test icp_alpha_shapes_test)
TARGET_LINK_LIBRARIES(icp_alpha_shapes_test CGAL scanlib  6dslam_core newmat_cmake dl brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util) #TODO: why is scanlib dl and newmat_camke not linked into 6dslam?!?

ADD_EXECUTABLE(icp_alpha_shapes_stream_test icp_alpha_shapes_stream_test)
TARGET_LINK_LIBRARIES(icp_alpha_shapes_stream_test CGAL scanlib  6dslam_core newmat_cmake dl brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)