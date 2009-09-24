# Locate OpenCV-1.0 install directory

INCLUDE (${PROJECT_SOURCE_DIR}/external/cmake_modules/FindPkg.cmake)

IF ( CMAKE_EPT_PKGCONFIG_EXECUTABLE )

    # Find all the opencv stuff with pkg-config
    EPT_PKGCONFIG( "opencv >= 1.0.0" OPENCV_FOUND OPENCV_INCLUDE_DIRS OPENCV_DEFINES OPENCV_LINK_DIRS OPENCV_LIBS )

    IF( OPENCV_FOUND )
        #MESSAGE("   Includes in: ${OPENCV_INCLUDE_DIRS}")
        #MESSAGE("   Libraries in: ${OPENCV_LINK_DIRS}")
        #MESSAGE("   Libraries: ${OPENCV_LIBS}")
        #MESSAGE("   Defines: ${OPENCV_DEFINES}")
    ENDIF ( OPENCV_FOUND )

ELSE  ( CMAKE_EPT_PKGCONFIG_EXECUTABLE )

    MESSAGE( "Can't find pkg-config" )

ENDIF ( CMAKE_EPT_PKGCONFIG_EXECUTABLE )