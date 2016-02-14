# Intertia tests
add_executable( rigidbody3d_inertia_tests rigidbody3d_inertia_tests.cpp )

target_link_libraries( rigidbody3d_inertia_tests rigidbody3d )
if( ENABLE_IWYU )
  set_property( TARGET rigidbody3d_inertia_tests PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

# Sphere inertia tests
add_test( rb3d_inertia_sphere_pool_ball rigidbody3d_inertia_tests sphere pool_ball )
# Box inertia tests
add_test( rb3d_inertia_box_0 rigidbody3d_inertia_tests box cube )
add_test( rb3d_inertia_box_1 rigidbody3d_inertia_tests box elongated )
add_test( rb3d_inertia_box_2 rigidbody3d_inertia_tests box elongated_two )
# Mesh inertia tests
#add_test( rigidbody3d_inertia_mesh_box_00 rigidbody3d_inertia_tests mesh box00 )
#add_test( rigidbody3d_inertia_mesh_box_01 rigidbody3d_inertia_tests mesh box01 )
#add_test( rigidbody3d_inertia_mesh_box_02 rigidbody3d_inertia_tests mesh box02 )


# Broad phase collision detection tests
add_executable( rigidbody3d_collision_detection_tests rigidbody3d_collision_detection_tests.cpp )

target_link_libraries( rigidbody3d_collision_detection_tests rigidbody3d )

add_test( rb3d_collision_detection_00 rigidbody3d_collision_detection_tests spatial_grid_00 )
add_test( rb3d_collision_detection_01 rigidbody3d_collision_detection_tests spatial_grid_01 )
add_test( rb3d_collision_detection_02 rigidbody3d_collision_detection_tests spatial_grid_02 )
