include( CMakeSourceFiles.txt )

add_library( rigidbody2d ${Headers} ${Sources} )

target_link_libraries( rigidbody2d scisim )
