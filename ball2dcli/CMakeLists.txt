include( CMakeSourceFiles.txt )

add_executable( ball2d_cli ${Headers} ${Sources} )
if( ENABLE_IWYU )
  set_property( TARGET ball2d_cli PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path} )
endif()

target_link_libraries( ball2d_cli ball2dutils ball2d )

execute_process( COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets/ball2d ${CMAKE_CURRENT_BINARY_DIR}/assets )


# Serialization and resume tests
if( USE_HDF5 )
  add_test( ball2d_serialization_00 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/ball_slide_at_threshold_big_dt.xml 0.5 50 27 100 )
  add_test( ball2d_serialization_01 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/everything_no_friction.xml 10.0 0999 0287 )
  add_test( ball2d_serialization_02 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/everything_friction.xml 10.0 0999 0287 )
  # Stabilized map tests
  add_test( ball2d_serialization_03 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/drift_safe_ball_on_plane.xml 4.5 45 22 10 )
  add_test( ball2d_serialization_04 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/drift_safe_balls_on_planes_00.xml 2.7 27 09 10 )
  add_test( ball2d_serialization_05 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/drift_safe_balls_on_planes_01.xml 3.0 30 07 10 )
  add_test( ball2d_serialization_06 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/drift_safe_symmetric_break.xml 24.0 240 012 10 )
  # Lees-Edwards boundary conditions tests
  add_test( ball2d_serialization_07 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/balls_collide_in_portal_test_00.xml 1.0 50 20 50 )
  add_test( ball2d_serialization_08 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/balls_collide_in_portal_test_01.xml 4.5 45 35 10 )
  add_test( ball2d_serialization_09 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/balls_collide_in_portal_test_02.xml 4.5 45 35 10 )
  add_test( ball2d_serialization_10 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/balls_collide_in_portal_test_03.xml 1.0 50 20 50 )
  add_test( ball2d_serialization_11 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/portal_collision_00.xml 4.0 40 25 10 )
  add_test( ball2d_serialization_12 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/portal_collision_01.xml 2.0 20 10 10 )
  add_test( ball2d_serialization_13 assets/shell_scripts/execute_serialization_test.sh assets/lees_edwards/portal_lots_of_collisions.xml 10.0 100 050 10 )
  # Warm-start tests
  add_test( ball2d_serialization_ws_00 assets/shell_scripts/execute_serialization_test.sh assets/tests_serialization/balls_falling_on_wedge.xml 4.0 40 20 10 )

  # These tests are slow
  #add_test( twod_balls_serialization_test_02 assets/shell_scripts/execute_serialization_test.sh assets/ColumnCollapse01.xml 0.04 2 4 120 )
  #add_test( twod_balls_serialization_test_04 assets/shell_scripts/execute_serialization_test.sh assets/ColumnCollapse07_frictionless_grr.xml 0.05 3 6 120 )
else()
  message( STATUS "Skipping Ball2D tests that require HDF5 (USE_HDF5 is disabled)." )
endif()

# Serialization tests for simulations that use Python callbacks
if( USE_HDF5 AND USE_PYTHON )
  add_test( ball2d_python_serialization_00 assets/shell_scripts/execute_serialization_test.sh assets/tests_python_serialization/oscillating_box.xml 20.0 2000 1113 100 )
  add_test( ball2d_python_serialization_01 assets/shell_scripts/execute_serialization_test.sh assets/tests_python_serialization/different_friction.xml 0.07 706 423 )
  add_test( ball2d_python_serialization_02 assets/shell_scripts/execute_serialization_test.sh assets/tests_python_serialization/wall_delete.xml 7.0 700 166 )
  add_test( ball2d_python_serialization_03 assets/shell_scripts/execute_serialization_test.sh assets/tests_python_serialization/different_restitution.xml 3.0 300 172 )
  add_test( ball2d_python_serialization_04 assets/shell_scripts/execute_serialization_test.sh assets/tests_python_serialization/insertion_test.xml 20.0 200 121 10 )
else()
  message( STATUS "Skipping Ball2D tests that require Python and HDF5 (USE_PYTHON or USE_HDF5 is disabled)." )
endif()

# Serialization tests for simulations that use Ipopt
if( USE_HDF5 AND USE_IPOPT )
  add_test( ball2d_ipopt_serialization_00 assets/shell_scripts/execute_serialization_test.sh assets/examples_gr/nine_ball_break_symmetric.xml 5.0 50 10 10 )
  add_test( ball2d_ipopt_serialization_01 assets/shell_scripts/execute_serialization_test.sh assets/examples_gr/butterfly.xml 30.0 300 100 10 )
else()
  message( STATUS "Skipping Ball2D tests that require Ipopt and HDF5 (USE_IPOPT or USE_HDF5 is disabled)." )
endif()

# Tests that require HDF5 support in the executable and the h5py python module
if( USE_HDF5 AND NOT H5PY_MISSING )
  # Static friction tests, plane pointing positive x
  add_test( ball2d_static_friction_posx_00 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_at_threshold.xml 3.0 1 )
  add_test( ball2d_static_friction_posx_01 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_below_threshold_05_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_posx_02 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_below_threshold_10_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_posx_03 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_below_threshold_15_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_posx_04 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_below_threshold_20_degree.xml 3.0 1 )

  # Static friction tests, plane pointing negative x
  add_test( ball2d_static_friction_negx_00 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_at_threshold.xml 3.0 1 )
  add_test( ball2d_static_friction_negx_01 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_below_threshold_05_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_negx_02 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_below_threshold_10_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_negx_03 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_below_threshold_15_degree.xml 3.0 1 )
  add_test( ball2d_static_friction_negx_03 assets/shell_scripts/static_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_below_threshold_20_degree.xml 3.0 1 )

  # Sliding friction tests, plane pointing positive x
  add_test( ball2d_sliding_friction_posx_00 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_above_threshold_05_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_posx_01 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_above_threshold_10_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_posx_02 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_above_threshold_15_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_posx_03 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_posx/ball_slide_above_threshold_20_degree.xml 3.0 1 )

  # Friction and sliding tests, plane pointing negative x
  add_test( ball2d_sliding_friction_negx_00 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_above_threshold_05_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_negx_01 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_above_threshold_10_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_negx_02 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_above_threshold_15_degree.xml 3.0 1 )
  add_test( ball2d_sliding_friction_negx_03 assets/shell_scripts/sliding_particle_test.sh assets/tests_friction/ball_plane/mu_0p5/slope_test_negx/ball_slide_above_threshold_20_degree.xml 3.0 1 )
else()
  message( STATUS "Skipping Ball2D tests that require HDF5 (USE_HDF5 is disabled) and the h5py Python module." )
endif()
