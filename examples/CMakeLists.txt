# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories
INCLUDE_DIRECTORIES(
    . 
    ${CMAKE_HOME_DIRECTORY}/src 
    ${OPENCV_INCLUDE_DIR}
    ${CMAKE_HOME_DIRECTORY}/external
    ${Boost_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${STANN_INCLUDE_DIRS}
    ${HDF5_CXX_INCLUDE_DIR}
)

IF (USE_EIGEN3)
	INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
ELSE (USE_EIGEN3)
	INCLUDE_DIRECTORIES(${EIGEN2_INCLUDE_DIR})
ENDIF (USE_EIGEN3)

LINK_DIRECTORIES(
   ${OpenCV_LINK_DIRECTORIES}
)

# compile and link the executables 
IF (USE_OPENCV)
    ADD_EXECUTABLE(depth_image_stream_test depth_image_stream_test)
    TARGET_LINK_LIBRARIES(depth_image_stream_test brics3d_core brics3d_algorithm brics3d_util)
    
    ADD_EXECUTABLE(depth_image_transformation depth_image_transformation)
    TARGET_LINK_LIBRARIES(depth_image_transformation brics3d_core brics3d_algorithm brics3d_util)
ENDIF(USE_OPENCV)

IF (USE_OSG)

    IF (USE_OPENCV)
        ADD_EXECUTABLE(BRICS_3D_Demo BRICS_3D_Demo)
        TARGET_LINK_LIBRARIES(BRICS_3D_Demo 
            brics3d_algorithm
            brics3d_util
            brics3d_core 
        	${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${OSGUTIL_LIBRARY_DEBUG}
        )
        
        ADD_EXECUTABLE(osg_point_plotter osg_point_plotter)
        TARGET_LINK_LIBRARIES(osg_point_plotter 
            brics3d_algorithm
            brics3d_util
            brics3d_core 
        	${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        )       
    ENDIF(USE_OPENCV)


    
    ADD_EXECUTABLE(osg_triangle_plotter osg_triangle_plotter)
    TARGET_LINK_LIBRARIES(osg_triangle_plotter 
        brics3d_algorithm
        brics3d_util
        brics3d_core
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
 
    ADD_EXECUTABLE(pointcloud_viewer pointcloud_viewer)
    TARGET_LINK_LIBRARIES(pointcloud_viewer
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    	  	    	    
    ADD_EXECUTABLE(icp_test icp_test) 
    TARGET_LINK_LIBRARIES(icp_test
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(icp_benchmark icp_benchmark) 
    TARGET_LINK_LIBRARIES(icp_benchmark
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(point3D_benchmark point3D_benchmark) 
    TARGET_LINK_LIBRARIES(point3D_benchmark 
        brics3d_algorithm
        brics3d_util
        brics3d_core 
        ${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    )	
     
    ADD_EXECUTABLE(demoSegmentationPlane demoSegmentationPlane)
    TARGET_LINK_LIBRARIES(demoSegmentationPlane 
        brics3d_algorithm
        brics3d_util
        brics3d_core 
        ${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
     ) 
     
    IF (USE_PCL AND USE_EIGEN3)	
        INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${OPENNI_INCLUDE_DIRS})

        ADD_EXECUTABLE(kinect_test kinect_test) 
        TARGET_LINK_LIBRARIES(kinect_test 
            brics3d_algorithm
            brics3d_util
            brics3d_core 
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
        
        ADD_EXECUTABLE(extract_object_clusters extract_object_clusters) 
        TARGET_LINK_LIBRARIES(extract_object_clusters 
            brics3d_algorithm
            brics3d_util
            brics3d_core 
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
               
        ADD_EXECUTABLE(wm_kinect_test wm_kinect_test) 
        TARGET_LINK_LIBRARIES(wm_kinect_test
            brics3d_world_model
            brics3d_algorithm
            brics3d_util
            brics3d_core 
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
        
        ADD_EXECUTABLE(wm_iterator_test wm_iterator_test) 
        TARGET_LINK_LIBRARIES(wm_iterator_test
            brics3d_world_model
            brics3d_algorithm
            brics3d_util
            brics3d_core 
            ${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${PCL_IO_LIBRARY}
        )
        
    ENDIF(USE_PCL AND USE_EIGEN3)    
    
    ADD_EXECUTABLE(wm_processing wm_processing)
    TARGET_LINK_LIBRARIES(wm_processing 
        brics3d_world_model
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    	${OSGUTIL_LIBRARY_DEBUG}
    	${OSGTEXT_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(wm_function_blocks wm_function_blocks)
    TARGET_LINK_LIBRARIES(wm_function_blocks 
        brics3d_world_model
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    	${OSGUTIL_LIBRARY_DEBUG}
    	${OSGTEXT_LIBRARY_DEBUG}
    )
    
    ADD_EXECUTABLE(wm_uncertain_transforms wm_uncertain_transforms)
    TARGET_LINK_LIBRARIES(wm_uncertain_transforms 
        brics3d_world_model
        brics3d_algorithm
        brics3d_util
        brics3d_core 
    	${OSG_LIBRARY_DEBUG}
    	${OSGGA_LIBRARY_DEBUG}
    	${OSGVIEWER_LIBRARY_DEBUG}
    	${OSGUTIL_LIBRARY_DEBUG}
    	${OSGTEXT_LIBRARY_DEBUG}
    )
   
    IF (USE_HDF5)
    message(STATUS "INFO: used HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
    ADD_EXECUTABLE(wm_hdf5_test wm_hdf5_test)
        TARGET_LINK_LIBRARIES(wm_hdf5_test
            brics3d_world_model
            brics3d_algorithm
            brics3d_util
            brics3d_core 
        	${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${OSGUTIL_LIBRARY_DEBUG}
        	${OSGTEXT_LIBRARY_DEBUG}
        	${HDF5_LIBRARIES}
        )   
    ADD_EXECUTABLE(wm_udp_hdf5_test wm_udp_hdf5_test)
     TARGET_LINK_LIBRARIES(wm_udp_hdf5_test
            brics3d_world_model
            brics3d_algorithm
            brics3d_util
            brics3d_core 
        	${OSG_LIBRARY_DEBUG}
        	${OSGGA_LIBRARY_DEBUG}
        	${OSGVIEWER_LIBRARY_DEBUG}
        	${OSGUTIL_LIBRARY_DEBUG}
        	${OSGTEXT_LIBRARY_DEBUG}
        	${HDF5_LIBRARIES}
        )     
    ENDIF(USE_HDF5)
    
#    ADD_EXECUTABLE(rsg_generated /home/sblume/eclipse_workspaces/runtime-EclipseXtext/WM/src-gen/rsg_main_generated.cpp)
#    TARGET_LINK_LIBRARIES(rsg_generated 
#        brics3d_world_model
#        brics3d_algorithm
#        brics3d_util
#        brics3d_core 
#    	${OSG_LIBRARY_DEBUG}
#    	${OSGGA_LIBRARY_DEBUG}
#    	${OSGVIEWER_LIBRARY_DEBUG}
#    	${OSGUTIL_LIBRARY_DEBUG}
#    )
ELSE(USE_OSG)

    #version as above but without OSG visualization
    IF (USE_HDF5)
    #message(STATUS "INFO: wm_hdf5_test_no_osg used HDF5_LIBRARIES: ${HDF5_LIBRARIES}")
    #ADD_EXECUTABLE(wm_hdf5_test_no_osg wm_hdf5_test)
    #TARGET_LINK_LIBRARIES(wm_hdf5_test_no_osg
    #        brics3d_world_model
    #        brics3d_algorithm
    #        brics3d_util
    #        brics3d_core 
    #    	${HDF5_LIBRARIES}
    #    )  
    #ADD_EXECUTABLE(wm_udp_hdf5_test_no_osg wm_udp_hdf5_test)
    #TARGET_LINK_LIBRARIES(wm_udp_hdf5_test_no_osg
    #        brics3d_world_model
    #        brics3d_algorithm
    #        brics3d_util
    #        brics3d_core 
    #    	${HDF5_LIBRARIES}
    #    )
    # TODO does not compile for now            
    ENDIF(USE_HDF5)
    
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
ADD_EXECUTABLE(demoSegmentationNormalPlane demoSegmentationNormalPlane)
TARGET_LINK_LIBRARIES(demoSegmentationNormalPlane brics3d_algorithm)

ADD_SUBDIRECTORY(wm_tutorial_1)

IF(USE_OSG)
ADD_SUBDIRECTORY(wm_tutorial_2)
ENDIF(USE_OSG)


IF(USE_JSON)
    INCLUDE_DIRECTORIES(/home/sblume/sandbox/json/libvariant/include)
    INCLUDE_DIRECTORIES(/home/sblume/sandbox/json/libvariant/build/include)      
    ADD_EXECUTABLE(wm_json wm_json)
    TARGET_LINK_LIBRARIES(wm_json 
         brics3d_world_model
         brics3d_core 
         brics3d_algorithm 
         brics3d_util 
        /home/sblume/sandbox/json/libvariant/build/src/libVariant.a 
        /usr/lib/x86_64-linux-gnu/libxml2.so
        /usr/lib/x86_64-linux-gnu/libyaml.so
        /usr/lib/x86_64-linux-gnu/libcurl.so
     )
ENDIF(USE_JSON)