include( CMakeSourceFiles.txt )

add_library( rigidbody3dutils ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET rigidbody3dutils PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

if( USE_IPOPT )
  set_source_files_properties( RigidBody3DSceneParser.cpp PROPERTIES COMPILE_DEFINITIONS IPOPT_FOUND )
endif()

find_package( RapidXML REQUIRED )
target_include_directories( rigidbody3dutils SYSTEM PRIVATE ${RAPIDXML_INCLUDE_DIR} )

target_link_libraries( rigidbody3dutils rigidbody3d )
