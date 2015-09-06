import sys
import h5py
import numpy
import argparse

parser = argparse.ArgumentParser( description='Determine if a simulation is static.' )
parser.add_argument( '-i', metavar='input_hdf5_file', type=str, nargs=1, help='input HDF5 file name', required=True )
args = parser.parse_args()

input_file_name = args.i[0]

print "Processing:", input_file_name

try:
  h5_file = h5py.File( input_file_name, 'r' )
  q = h5_file['q'][:]
  v = h5_file['v'][:]
  h5_file.close()
except:
  print 'Failed to read in HDF5 file, exiting.'
  sys.exit(1)

# Can tighten these tolerances further by decreasing the time step, but then the simulations are slow
if abs( q[0] ) > 1.0e-12 :
  print "First q component incorect."
  sys.exit(1)
if abs( q[1] ) > 3.0e-4 :
  print "Second q component incorect."
  sys.exit(1)
if abs( v[0] ) > 1.0e-12 : 
  print "First v component incorrect."
  sys.exit(1)
if abs( v[1] ) > 2.0e-4 : 
  print "Second v component incorrect."
  sys.exit(1)

sys.exit(0)
