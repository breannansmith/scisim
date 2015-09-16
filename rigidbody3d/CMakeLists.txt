include( CMakeSourceFiles.txt )

add_library( rigidbody3d ${Headers} ${Sources} )

target_link_libraries( rigidbody3d scisim )
