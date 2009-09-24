# author: Sebastian Blumenthal
# date: 2009-09-09

# add include directories (-I)
INCLUDE_DIRECTORIES(
    . 
    #${CMAKE_HOME_DIRECTORY}/src/newmat
)


# add library directories (-L)
LINK_DIRECTORIES(
    #${CMAKE_HOME_DIRECTORY}/src/newmat
    #${GUI_LIBRARY_LIB_INCLUDES}	
)


# define sources lists for libraries (cpp/cc files)
SET(NEWMAT_LIBRARY_SOURCES
	newmat/newmat1
	newmat/newmat2
	newmat/newmat3
	newmat/newmat4
	newmat/newmat5
	newmat/newmat6
	newmat/newmat7
	newmat/newmat8
	newmat/newmatex
	newmat/bandmat
	newmat/submat
	newmat/myexcept
	newmat/cholesky
	newmat/evalue
	newmat/fft
	newmat/hholder
	newmat/jacobi
	newmat/newfft
	newmat/sort
	newmat/svd
	newmat/newmatrm
	newmat/newmat9
)

SET(SCANLIB_LIBRARY_SOURCES
	octtree
	kd
	kdc
	scan
	d2tree
)

SET(CORE_LIBRARY_SOURCES
	icp6D
	graphSlam6D 
	icp6Dapx
	icp6Dsvd
	icp6Dortho
	icp6Dquat
	icp6Dhelix
	gapx6D
	ghelix6DQ2
	lum6Deuler
	lum6Dquat
	graph
	#slam6D #?
	globals.icc
	sparse/csparse 
	elch6D
	elch6Dquat
	elch6DunitQuat
	elch6Dslerp
	elch6Deuler
)

# shared file IO (Scan) libraries(TODO: port all)
#$(BIN)scan_io_uos.so: $(SRC)scan_io.h $(SRC)scan_io_uos.h $(SRC)scan_io_uos.cc $(SRC)point.h $(SRC)point.icc $(SRC)globals.icc
#	echo Compiling shared library for reading UOS scans ...
#	$(GPP) $(CFLAGS) $(SHAREDFLAGS) -o $(BIN)scan_io_uos.so $(SRC)scan_io_uos.cc 
SET(SCAN_IO_UOS_LIBRARY
    scan_io_uos
    point.icc 
    globals.icc
)

# compile libraries
ADD_LIBRARY(newmat_cmake ${NEWMAT_LIBRARY_SOURCES})
ADD_LIBRARY(scanlib ${SCANLIB_LIBRARY_SOURCES})
ADD_LIBRARY(6dslam_core ${CORE_LIBRARY_SOURCES})
TARGET_LINK_LIBRARIES(6dslam_core newmat_cmake scanlib dl)

# compile shared file IO (Scan) libraries
ADD_LIBRARY(scan_io_uos SHARED ${SCAN_IO_UOS_LIBRARY})

# compile binaries
ADD_EXECUTABLE(slam6D slam6D)
TARGET_LINK_LIBRARIES(slam6D 6dslam_core)






