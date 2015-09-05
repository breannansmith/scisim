include( CMakeSourceFiles.txt )

add_library( ball2d ${Headers} ${Sources} )

target_link_libraries( ball2d scisim )
