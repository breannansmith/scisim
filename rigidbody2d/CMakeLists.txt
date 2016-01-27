include( CMakeSourceFiles.txt )

add_library( rigidbody2d ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET rigidbody2d PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

target_link_libraries( rigidbody2d scisim )
