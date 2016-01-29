include( CMakeSourceFiles.txt )

add_library( rigidbody3d ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET rigidbody3d PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

target_link_libraries( rigidbody3d scisim )
