# - Try to find MICROBLX
# Once done this will define
#
#  MICROBLX_FOUND - if MICROBLX was found
#  MICROBLX_INCLUDE_DIR - the MICROBLX include directory (-I)
#  MICROBLX_LINK_DIRECTORIES - the MICROBLX linker directories (-L)
#  MICROBLX_LIBRARIES - MICROBLX libraries
#
# You can set an environment variable "MICROBLX_DIR" to help CMake to find the MICROBLX library,
# in case it is not installed in one of the standard paths.
#

FIND_PATH(MICROBLX_INCLUDE_DIR NAMES ubx.h
  PATHS
  $ENV{MICROBLX_DIR}/src
  ENV CPATH
  /usr/include/
  /usr/local/include/
  NO_DEFAULT_PATH
)

FIND_LIBRARY(MICROBLX_LIBRARY NAMES "ubx" 
  PATHS
  $ENV{MICROBLX_DIR}/src
  $ENV{MICROBLX_DIR}/lib 
  ENV LD_LIBRARY_PATH
  ENV LIBRARY_PATH
  /usr/lib
  /usr/local/lib
  NO_DEFAULT_PATH
)

IF(MICROBLX_LIBRARY)
  GET_FILENAME_COMPONENT(MICROBLX_LINK_DIRECTORIES ${MICROBLX_LIBRARY} PATH CACHE)
ENDIF(MICROBLX_LIBRARY)


SET(MICROBLX_LIBRARIES_TMP
)


IF(MICROBLX_LIBRARY) 
  SET(MICROBLX_LIBRARIES
    ${MICROBLX_LIBRARY}
    CACHE STRING "Mixroblx library"
  ) 
ENDIF(MICROBLX_LIBRARY)





include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set MICROBLX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(MICROBLX  DEFAULT_MSG
                                  MICROBLX_INCLUDE_DIR MICROBLX_LINK_DIRECTORIES MICROBLX_LIBRARIES)

# show the MICROBLX_INCLUDE_DIR and MICROBLX_LIBRARIES variables only in the advanced view
IF(MICROBLX_FOUND)
  MARK_AS_ADVANCED(MICROBLX_INCLUDE_DIR MICROBLX_LINK_DIRECTORIES MICROBLX_LIBRARIES)
ENDIF(MICROBLX_FOUND)

