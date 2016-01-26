include( CMakeSourceFiles.txt )

add_library( ball2d ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET ball2d PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

target_link_libraries( ball2d scisim )
