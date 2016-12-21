# Broad phase tests
add_executable( collision_detection_tests collision_detection_tests.cpp )
if( ENABLE_IWYU )
  set_property( TARGET collision_detection_tests PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

target_link_libraries( collision_detection_tests ball2d )

add_test( ball2d_collision_detection_00 collision_detection_tests spatial_grid_00 )
add_test( ball2d_collision_detection_01 collision_detection_tests spatial_grid_01 )
add_test( ball2d_collision_detection_02 collision_detection_tests spatial_grid_02 )
