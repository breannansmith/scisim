# Created:
# IPOPT_INC_DIRS - Directories to include to use IPOPT
# IPOPT_LIB      - Library to link against to use IPOPT
# IPOPT_FOUND    - If false, don't try to use IPOPT

# NOTE NOTE NOTE NOTE NOTE
# This file uses the ipopt_addlibs_cpp.txt file

set( IPOPT_DIR $ENV{IPOPT_DIR} )

if( NOT IPOPT_DIR )
  message( FATAL_ERROR "Please set the environment variable IPOPT_DIR to point to your Ipopt installation." )
endif()

# TODO: Actually verify that the proper files are here
set( IPOPT_INC_DIRS ${IPOPT_DIR}/include/coin )

find_library( IPOPT_LIB ipopt ${IPOPT_DIR}/lib ${IPOPT_DIR}/lib/coin NO_DEFAULT_PATH )

if( IPOPT_LIB )
  # Determine the additional libraries that Ipopt must be linked against.
  # Luckily, the Ipopt build process now generates these for us.
  file( READ ${IPOPT_DIR}/share/coin/doc/Ipopt/ipopt_addlibs_cpp.txt IPOPT_DEP )
  
  if( NOT IPOPT_DEP )
    message( FATAL_ERROR "Failed to locate the ipopt_addlibs_cpp.txt file at: ${IPOPT_DIR}/share/coin/doc/Ipopt/ipopt_addlibs_cpp.txt" )
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
  set( IPOPT_INC_DIRS "" )
  set( IPOPT_LIB "" )
endif()

mark_as_advanced(IPOPT_INC_DIRS)
mark_as_advanced(IPOPT_LIB)
mark_as_advanced(IPOPT_FOUND)
