include( CMakeSourceFiles.txt )

add_library( ball2dutils ${Headers} ${Sources} )

find_package( RapidXML REQUIRED )
target_include_directories( ball2dutils SYSTEM PRIVATE ${RAPIDXML_INCLUDE_DIR} )

target_link_libraries( ball2dutils ball2d )
