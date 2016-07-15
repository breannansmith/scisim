# Created:
# IPOPT_INC_DIRS - Directories to include to use Ipopt
# IPOPT_LIB      - Library to link against to use Ipopt
# IPOPT_DOC_PATH - Directory containing Ipopt link instructions
# IPOPT_FOUND    - If false, don't try to use Ipopt

# NOTE NOTE NOTE NOTE NOTE
# This file uses the ipopt_addlibs_cpp.txt file

find_library( IPOPT_LIB ipopt )
if( NOT IPOPT_LIB )
  message( FATAL_ERROR "Failed to find the Ipopt library. Ensure that the path to Ipopt is in CMAKE_PREFIX_PATH." )
endif()

find_path( IPOPT_INC_DIRS IpNLP.hpp PATH_SUFFIXES include/coin )
if( NOT IPOPT_INC_DIRS )
  message( FATAL_ERROR "Failed to find the Ipopt header include path. Ensure that the path to Ipopt is in CMAKE_PREFIX_PATH." )
endif()

find_path( IPOPT_DOC_PATH ipopt_addlibs_cpp.txt PATH_SUFFIXES share/coin/doc/Ipopt )
if( NOT IPOPT_DOC_PATH )
  message( FATAL_ERROR "Failed to find the Ipopt doc path. Ensure that the path to Ipopt is in CMAKE_PREFIX_PATH." )
endif()

if( IPOPT_LIB )
  # Determine the additional libraries that Ipopt must be linked against.
  # Luckily, the Ipopt build process now generates these for us.
  file( READ ${IPOPT_DOC_PATH}/ipopt_addlibs_cpp.txt IPOPT_DEP )
  
  if( NOT IPOPT_DEP )
    message( FATAL_ERROR "Failed to read the ipopt_addlibs_cpp.txt file at: ${IPOPT_DOC_PATH}/ipopt_addlibs_cpp.txt" )
  endif()

  # TODO: Should the list of libraries have the -L and -l stuff stripped?
  #STRING(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
  #STRING(REPLACE "-l" "" IPOPT_DEP ${IPOPT_DEP})
  
  # Strip any newlines from the library list.
  string( REPLACE "\n" "" IPOPT_DEP ${IPOPT_DEP} )
  string( STRIP "${IPOPT_DEP}" IPOPT_DEP )

  # Convert to a list at spaces
  separate_arguments( IPOPT_DEP )

  # Add entries to the library list 
  list( LENGTH IPOPT_DEP Ipopt_LIST_LEN )
  set( argnum 0 )
  while( argnum LESS ${Ipopt_LIST_LEN} )
    list( GET IPOPT_DEP ${argnum} current_argument )

    # If the current argument is a framework
    if( current_argument STREQUAL "-framework" )
      # The next argument is the framework name
      math( EXPR argnum "${argnum} + 1" )
      list( GET IPOPT_DEP ${argnum} next_argument )
      # Save the framework
      list( APPEND IPOPT_LIB "${current_argument} ${next_argument}" )
    else()
      # Save the library
      list( APPEND IPOPT_LIB ${current_argument} )
    endif()

    math( EXPR argnum "${argnum} + 1" )
  endwhile()

endif()

if( IPOPT_LIB )
  set( IPOPT_FOUND TRUE )
else()
  set( IPOPT_FOUND FALSE )
  set( IPOPT_LIB "" )
  set( IPOPT_INC_DIRS "" )
  set( IPOPT_DOC_PATH "" )
endif()

mark_as_advanced(IPOPT_LIB)
mark_as_advanced(IPOPT_INC_DIRS)
mark_as_advanced(IPOPT_DOC_PATH)
mark_as_advanced(IPOPT_FOUND)
