include( CMakeSourceFiles.txt )

if( APPLE )
  set( MACOSX_BUNDLE_ICON_FILE SCISim )
  set( AppIcon ${PROJECT_SOURCE_DIR}/resources/SCISim.icns )
  set_source_files_properties( ${AppIcon} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources" )
  add_executable( ball2d_qt5 MACOSX_BUNDLE ${AppIcon} ${Headers} ${Sources} ${HWQ_MOC_CPP} )
  set_target_properties( ball2d_qt5 PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${PROJECT_SOURCE_DIR}/resources/Info.plist_ball2d_qt5 )
  target_compile_definitions( ball2d_qt5 PRIVATE USE_MACOS )
else()
  add_executable( ball2d_qt5 ${Headers} ${Sources} ${HWQ_MOC_CPP} )
endif()

if( ENABLE_IWYU )
  set_property( TARGET ball2d_qt5 PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

set_target_properties( ball2d_qt5 PROPERTIES AUTOMOC TRUE )

find_package( OpenGL REQUIRED )
target_include_directories( ball2d_qt5 SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR} )

find_package( Qt5 COMPONENTS Widgets OpenGL REQUIRED )

target_link_libraries( ball2d_qt5 ball2dutils ball2d ${OPENGL_LIBRARIES} Qt5::Widgets Qt5::OpenGL )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/ball2d ${CMAKE_CURRENT_BINARY_DIR}/assets )
