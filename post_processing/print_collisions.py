'''Simple example of CollisionState usage that prints the collisions in a system.'''

import sys
import os
import h5py
import rb3d_processing

if len(sys.argv) != 2:
    sys.exit('Usage: python {} hdf5_file'.format(sys.argv[0]))

hdf5_file_name = sys.argv[1]

if not os.path.isfile(hdf5_file_name):
    sys.exit('Error, file {} does not exist.'.format(hdf5_file_name))

try:
    with h5py.File(hdf5_file_name, 'r') as h5_file:
        collision_state = rb3d_processing.CollisionState(h5_file)

    print 'Git hash:', collision_state.git_hash
    print 'Iteration:', collision_state.iteration
    print 'Timestep:', collision_state.timestep
    print 'Time:', collision_state.time
    for idx, (impulse, indices, normal, point) in enumerate(collision_state.collisions()):
        print 'Collision:', idx
        print '  impulse:', impulse
        print '  indices:', indices
        print '  normal:', normal
        print '  point:', point
except IOError as io_exception:
    sys.exit(str(io_exception))
