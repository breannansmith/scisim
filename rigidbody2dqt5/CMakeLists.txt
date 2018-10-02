include( CMakeSourceFiles.txt )

if( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
  set( MACOSX_BUNDLE_ICON_FILE SCISim )
  set( AppIcon ${PROJECT_SOURCE_DIR}/resources/SCISim.icns )
  set_source_files_properties( ${AppIcon} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources" )
  add_executable( rigidbody2d_qt5 MACOSX_BUNDLE ${AppIcon} ${Headers} ${Sources} ${HWQ_MOC_CPP} )
  set_target_properties( rigidbody2d_qt5 PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${PROJECT_SOURCE_DIR}/resources/Info.plist_rigidbody2d_qt5 )
else()
  add_executable( rigidbody2d_qt5 ${Headers} ${Sources} ${HWQ_MOC_CPP} )
endif()

if( ENABLE_IWYU )
  set_property( TARGET rigidbody2d_qt5 PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

set_target_properties( rigidbody2d_qt5 PROPERTIES AUTOMOC TRUE )

find_package( OpenGL REQUIRED )
target_include_directories( rigidbody2d_qt5 SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR} )

find_package( Qt5 COMPONENTS Widgets OpenGL REQUIRED )

target_link_libraries( rigidbody2d_qt5 rigidbody2dutils rigidbody2d ${OPENGL_LIBRARIES} Qt5::Widgets Qt5::OpenGL )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/rigidbody2d ${CMAKE_CURRENT_BINARY_DIR}/assets )
