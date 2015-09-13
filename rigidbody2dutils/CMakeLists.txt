include( CMakeSourceFiles.txt )

add_library( rigidbody2dutils ${Headers} ${Sources} )

find_package( RapidXML REQUIRED )
target_include_directories( rigidbody2dutils SYSTEM PRIVATE ${RAPIDXML_INCLUDE_DIR} )

target_link_libraries( rigidbody2dutils rigidbody2d )
