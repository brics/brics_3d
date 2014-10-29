# add include directories
INCLUDE_DIRECTORIES(
    ${CMAKE_HOME_DIRECTORY}/src 
    ${CMAKE_HOME_DIRECTORY}/external
    ${CPPUNIT_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
    ${STANN_INCLUDE_DIRS}
)

IF (USE_EIGEN3)
	INCLUDE_DIRECTORIES( ${EIGEN_INCLUDE_DIRS})
ELSE (USE_EIGEN3)
	INCLUDE_DIRECTORIES( ${EIGEN2_INCLUDE_DIR})
ENDIF (USE_EIGEN3)

IF (USE_HDF5)
	INCLUDE_DIRECTORIES( 
	    ${HDF5_CXX_INCLUDE_DIR}
	    #${Boost_INCLUDE_DIR}
	 )
	SET(HDF5_LIBS ${HDF5_LIBRARIES})
ELSE(USE_HDF5)
	SET(HDF5_LIBS "")	
ENDIF (USE_HDF5)

# add library directories (-L)
LINK_DIRECTORIES(

)


# automatically search for cppUnit tests in this folder
# NOTE: all test must end with Test.cpp e.g. Point3DTest.cpp
FILE(GLOB CPPUNITTEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*Test.cpp" )
MESSAGE("CppUnitTests: ${CPPUNITTEST_SOURCES}")

# compile unit testing executable 
ADD_EXECUTABLE(run_unit_tests TestRunner.cpp ${CPPUNITTEST_SOURCES})
#TARGET_LINK_LIBRARIES(run_unit_tests cppunit brics3d_core)
TARGET_LINK_LIBRARIES(run_unit_tests ${CPPUNIT_LIBRARIES} brics3d_core brics3d_algorithm brics3d_util brics3d_world_model ${HDF5_LIBS})


# compile unit testing executable for ECUT eclipse plugin
ADD_DEFINITIONS(-DECUT_MAIN=main)
ADD_EXECUTABLE(run_ecut_unit_tests ECUTTestRunner.cpp ${CPPUNITTEST_SOURCES})
#TARGET_LINK_LIBRARIES(run_ecut_unit_tests cppunit brics3d_core)
TARGET_LINK_LIBRARIES(run_ecut_unit_tests ${CPPUNIT_LIBRARIES} brics3d_core brics3d_algorithm brics3d_util brics3d_world_model ${HDF5_LIBS})


