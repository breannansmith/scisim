include( CMakeSourceFiles.txt )

add_executable( rigidbody2d_qt4 ${Headers} ${Sources} ${HWQ_MOC_CPP} )
if( ENABLE_IWYU )
  set_property( TARGET rigidbody2d_qt4 PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

set_target_properties( rigidbody2d_qt4 PROPERTIES AUTOMOC TRUE )

find_package( OpenGL REQUIRED )
target_include_directories( rigidbody2d_qt4 SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR} )

find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
include( ${QT_USE_FILE} )

target_link_libraries( rigidbody2d_qt4 ${QT_LIBRARIES} ${OPENGL_LIBRARIES} rigidbody2dutils rigidbody2d )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/rigidbody2d ${CMAKE_CURRENT_BINARY_DIR}/assets )
