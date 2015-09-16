include( CMakeSourceFiles.txt )

add_library( rigidbody3dutils ${Headers} ${Sources} )

find_package( RapidXML REQUIRED )
target_include_directories( rigidbody3dutils SYSTEM PRIVATE ${RAPIDXML_INCLUDE_DIR} )

target_link_libraries( rigidbody3dutils rigidbody3d )
