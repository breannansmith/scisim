include( CMakeSourceFiles.txt )

add_executable( ball2d_qt4 ${Headers} ${Sources} ${HWQ_MOC_CPP} )

set_target_properties( ball2d_qt4 PROPERTIES AUTOMOC TRUE )

find_package( OpenGL REQUIRED )
target_include_directories( ball2d_qt4 SYSTEM PRIVATE ${OPENGL_INCLUDE_DIR} )

find_package( Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED )
include( ${QT_USE_FILE} )

target_link_libraries( ball2d_qt4 ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ball2dutils ball2d )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/2DBallScenes ${CMAKE_CURRENT_BINARY_DIR}/assets )
