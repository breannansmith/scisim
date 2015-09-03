# - Try to find the So-bogus library (https://bitbucket.org/gdaviet/so-bogus)
#
# Once done this will define
#
#  SOBOGUS_FOUND - So-bogus was found
#  SOBOGUS_INCLUDE_DIR - the So-bogus include directory

# Breannan Smith (smith@cs.columbia.edu)

find_path( SOBOGUS_INCLUDE_DIR NAMES Core/BlockSolvers.hpp
    PATHS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_INSTALL_PREFIX}/include
    PATH_SUFFIXES sobogus
  )

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Sobogus DEFAULT_MSG SOBOGUS_INCLUDE_DIR )
mark_as_advanced( SOBOGUS_INCLUDE_DIR )
