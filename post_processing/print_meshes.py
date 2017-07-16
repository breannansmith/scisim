'''Simple example of DiscreteState usage that prints the mesh bodies in a system.'''

import sys
import os
import h5py
import rb3d_processing

if len(sys.argv) != 2:
    sys.exit('Usage: python {} hdf5_file'.format(sys.argv[0]))

hdf5_file_name = sys.argv[1]

if not os.path.isfile(hdf5_file_name):
    sys.exit('Error, file {} does not exist.'.format(hdf5_file_name))


def printPaddedArray(A):
    '''Prints an array with spaces padded to the front.'''
    assert A.shape == (3, 3)
    print '[{} {} {}]'.format(A[0, 0], A[0, 1], A[0, 2])
    print '     [{} {} {}]'.format(A[1, 0], A[1, 1], A[1, 2])
    print '     [{} {} {}]'.format(A[2, 0], A[2, 1], A[2, 2])


try:
    with h5py.File(hdf5_file_name, 'r') as h5_file:
        sim_state = rb3d_processing.DiscreteState(h5_file)

    print 'Git hash:', sim_state.git_hash
    print 'Iteration:', sim_state.iteration
    print 'Timestep:', sim_state.timestep
    print 'Time:', sim_state.time
    for idx, (x, R, vel, omega, M, I, kinematic, mesh_name) in enumerate(sim_state.meshBodies()):
        print 'Body:', idx
        print '  x:', x
        print '  R:',
        printPaddedArray(R)
        print '  v:', vel
        print '  omega:', omega
        print '  M:', M
        print '  I:',
        printPaddedArray(I)
        print '  fixed:', kinematic
        print '  mesh_name:', mesh_name
except IOError as io_exception:
    sys.exit(str(io_exception))
