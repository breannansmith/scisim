''' Checks whether a simulation that should be static is static. '''

import os
import sys
import argparse
import h5py

parser = argparse.ArgumentParser( description='Checks whether a simulation that should be static is static.' )
parser.add_argument( '-i', metavar='input_hdf5_file', type=str, nargs=1, help='input HDF5 file name', required=True )
args = parser.parse_args()

input_file_name = args.i[0]
if not os.path.isfile(input_file_name):
  sys.exit('Error, input file \'' + input_file_name + '\' does not exist.')

print 'Validating:', input_file_name

try:
  with h5py.File( input_file_name, 'r' ) as h5_file:
    q = h5_file['q'][:]
    v = h5_file['v'][:]
except IOError as io_exception:
  sys.exit('HDF5 IO Error: ' + io_exception.message)
except KeyError as key_exception:
  sys.exit('HDF5 Key Error: ' + key_exception.message)

# Can tighten these tolerances further by decreasing the time step, but then the simulations are slow
if abs( q[0] ) > 3.0e-11:
  sys.exit('First q component incorect')
if abs( q[1] ) > 3.0e-3:
  sys.exit('Second q component incorect')
# TODO: Grab the plane, test that the orientation is consistent with plane
# if abs( q[2] ) > 2e-1:
#   sys.exit('Third q component incorect')
if abs( v[0] ) > 2.0e-11:
  sys.exit('First v component incorrect')
if abs( v[1] ) > 2.0e-3:
  sys.exit('Second v component incorrect')
if abs( v[2] ) > 1.0e-12:
  sys.exit('Third v component incorrect')

sys.exit(0)
