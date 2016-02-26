# 3.2.1 to allow set_property( TARGET target PROPERTY CXX_STANDARD 11 ) and similar
cmake_minimum_required( VERSION 3.2.1 )

# Check that the Eigen is installed
execute_process( COMMAND ./scripts_include/eigen_found.sh RESULT_VARIABLE EIGEN_NOT_FOUND WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
if( EIGEN_NOT_FOUND )
  message( FATAL_ERROR "Failed to locate Eigen or checksum is incorrect. Please run get_dependencies.sh." )
endif()

# Check that RapidXml is installed
execute_process( COMMAND ./scripts_include/rapidxml_found.sh RESULT_VARIABLE RAPIDXML_NOT_FOUND WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
if( RAPIDXML_NOT_FOUND )
  message( FATAL_ERROR "Failed to locate RapidXml or checksum is incorrect. Please run get_dependencies.sh." )
endif()

# Check that So-bogus is installed
execute_process( COMMAND ./scripts_include/sobogus_found.sh RESULT_VARIABLE SOBOGUS_NOT_FOUND WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )
if( SOBOGUS_NOT_FOUND )
  message( FATAL_ERROR "Failed to locate So-bogus or checksum is incorrect. Please run get_dependencies.sh." )
endif()

# Enable Fortran if the user provided a Fortran compiler
if( "$ENV{FC}" STREQUAL "" AND NOT CMAKE_Fortran_COMPILER )
  project( SCISim C CXX )
else()
  project( SCISim C CXX Fortran )
endif()


################################################################################
### Configure and check the build setting options

# If the user provided a CMAKE_BUILD_TYPE, create a cache variable
if( NOT CMAKE_BUILD_TYPE STREQUAL "" )
  set( CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
       build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
       Release RelWithDebInfo MinSizeRel." FORCE )
# Otherwise default to Release
else( NOT CMAKE_BUILD_TYPE STREQUAL "" )
  set( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of
       build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
       Release RelWithDebInfo MinSizeRel." FORCE )
endif( NOT CMAKE_BUILD_TYPE STREQUAL "" )

# Enforce the use of a valid build type
if( NOT CMAKE_BUILD_TYPE STREQUAL "None" AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  if( NOT CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
    if( NOT CMAKE_BUILD_TYPE STREQUAL "MinSizeRel" )
      message( SEND_ERROR "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." )
    endif( NOT CMAKE_BUILD_TYPE STREQUAL "MinSizeRel" )
  endif( NOT CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
endif( NOT CMAKE_BUILD_TYPE STREQUAL "None" AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )


################################################################################
### Set flags used across all builds

# Disable compiler language extensions
set( CMAKE_CXX_EXTENSIONS OFF )
# Enable C++14
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# Append an extra space if flags are already set
if( CMAKE_CXX_FLAGS )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} " )
endif( CMAKE_CXX_FLAGS )

# Allow the user to optionally enable compiler sanitizers
set( SANITIZER "none" CACHE STRING "Enable compiler sanitizer modes, options are: none, address" )

# Enable compiler specific flags
if( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" )
  include( CMakeListsGNUCXXFlags.txt )
elseif( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel" )
  include( CMakeListsIntelCXXFlags.txt )
elseif( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
  include( CMakeListsClangCXXFlags.txt )
elseif( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" )
  # For now, same options to Clang and AppleClang. May change in the future.
  include( CMakeListsClangCXXFlags.txt )
else()
  message( FATAL_ERROR "Unsupported C++ compiler ${CMAKE_CXX_COMPILER_ID}. Please email the maintainer." )
endif()


################################################################################
### Set paths, etc

# Create a default location for custom CMake modules
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake )

# Allow projects to reference source relative to the root of the project
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )


################################################################################
### Add options to allow the user to configure the build

option( STRICT_BUILD "Enable aggressive warnings and treat warnings as errors" OFF )

option( ENABLE_IWYU "Enables builds with 'include-what-you-use' (http://include-what-you-use.org/)" )

option( USE_QT4 "Build Qt4 front ends" OFF )

option( USE_IPOPT "Use the Ipopt solver (Fortran required)" OFF )

option( USE_OPENMP "OpenMP-based parallelism" OFF )

option( USE_HDF5 "Serialization support with HDF5" OFF )

option( USE_PYTHON "Embedded Python language scripting" OFF )

###############################################################################
### Check for installed Python modules

execute_process( COMMAND python -c "import h5py" RESULT_VARIABLE H5PY_MISSING ERROR_QUIET )
execute_process( COMMAND python -c "import numpy" RESULT_VARIABLE NUMPY_MISSING ERROR_QUIET )

###############################################################################
### Add libraries, plugins

if( ENABLE_IWYU )
  find_program( iwyu_path NAMES include-what-you-use iwyu )
  if( NOT iwyu_path )
    message( FATAL_ERROR "Failed to locate include-what-you-use." )
  endif()
endif()

# Allow targets to generate tests
enable_testing()

# Emit an error if the user requested a feature that needs Fortran but did not provide a Fortran compiler
if( NOT CMAKE_Fortran_COMPILER )
  if( USE_IPOPT )
    message( FATAL_ERROR "Error, Ipopt requires Fortran. Pleae provide a Fortran compiler." )
  endif()
  if( EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scisim/Math/QL/ql.for )
    message( FATAL_ERROR "Error, QL requires Fortran. Pleae provide a Fortran compiler." )
  endif()
endif()

# Core SCISim library
add_subdirectory( scisim )

# Tests for the SCISim library
add_subdirectory( scisimtests )


# Core two dimensional ball simulation library
add_subdirectory( ball2d )

# Tests for the two dimensional ball simulation library
add_subdirectory( ball2dtests )

# Utilities shared by the command-line and Qt4 interface
add_subdirectory( ball2dutils )

# Command line interface for two dimensional ball simulation
add_subdirectory( ball2dcli )

# Qt4 interface for two dimensional ball simulation
if( USE_QT4 )
  add_subdirectory( ball2dqt4 )
endif()


# Core two dimensional rigid body library
add_subdirectory( rigidbody2d )

# Utilities shared by the command-line and Qt4 interface
add_subdirectory( rigidbody2dutils )

# Command line interface for two dimensional rigid body simulations
add_subdirectory( rigidbody2dcli )

# Qt4 interface for two dimensional rigid body simulations
if( USE_QT4 )
  add_subdirectory( rigidbody2dqt4 )
endif()


# Core three dimensional rigid body simulation library
add_subdirectory( rigidbody3d )

# Tets for the three dimensional rigid body simulation
add_subdirectory( rigidbody3dtests )

# Utilities shared by the command-line and Qt4 interface
add_subdirectory( rigidbody3dutils )

# Command line interface for three dimensional rigid body simulation
add_subdirectory( rigidbody3dcli )

# Qt4 interface for the three dimensional rigid body simulation
if( USE_QT4 )
  add_subdirectory( rigidbody3dqt4 )
endif()
