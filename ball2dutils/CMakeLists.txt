include( CMakeSourceFiles.txt )

add_library( ball2dutils ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET ball2dutils PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

if( USE_IPOPT )
  set_source_files_properties( Ball2DSceneParser.cpp PROPERTIES COMPILE_DEFINITIONS IPOPT_FOUND )
endif()

find_package( RapidXML REQUIRED )
target_include_directories( ball2dutils SYSTEM PRIVATE ${RAPIDXML_INCLUDE_DIR} )

target_link_libraries( ball2dutils ball2d )
