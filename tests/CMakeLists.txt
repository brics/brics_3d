# add include directories
INCLUDE_DIRECTORIES(
    ${CMAKE_HOME_DIRECTORY}/src 
    ${CMAKE_HOME_DIRECTORY}/external
    ${EIGEN2_INCLUDE_DIR}
    ${CPPUNIT_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
)

# add library directories (-L)
LINK_DIRECTORIES(
    ${CMAKE_HOME_DIRECTORY}/external/6dslam/lib
)


# automatically search for cppUnit tests in this folder
# NOTE: all test must end with Test.cpp e.g. Point3DTest.cpp
FILE(GLOB CPPUNITTEST_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*Test.cpp" )
MESSAGE("CppUnitTests: ${CPPUNITTEST_SOURCES}")

# compile unit testing executable 
ADD_EXECUTABLE(run_unit_tests TestRunner.cpp ${CPPUNITTEST_SOURCES})
#TARGET_LINK_LIBRARIES(run_unit_tests cppunit brics_3dpm_core)
TARGET_LINK_LIBRARIES(run_unit_tests ${CPPUNIT_LIBRARIES} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)


# compile unit testing executable for ECUT eclipse plugin
ADD_DEFINITIONS(-DECUT_MAIN=main)
ADD_EXECUTABLE(run_ecut_unit_tests ECUTTestRunner.cpp ${CPPUNITTEST_SOURCES})
#TARGET_LINK_LIBRARIES(run_ecut_unit_tests cppunit brics_3dpm_core)
TARGET_LINK_LIBRARIES(run_ecut_unit_tests ${CPPUNIT_LIBRARIES} brics_3dpm_core brics_3dpm_algorithm brics_3dpm_util)


