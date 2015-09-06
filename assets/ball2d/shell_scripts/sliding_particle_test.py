import sys
import h5py
import numpy
import argparse
import math

parser = argparse.ArgumentParser( description='Check if the position of a sliding particle on a plane is consistent with theory.' )
parser.add_argument( '-i', metavar='input_hdf5_file', type=str, nargs=1, help='input HDF5 file name', required=True )
args = parser.parse_args()

input_file_name = args.i[0]

print "Processing:", input_file_name

try:
  h5_file = h5py.File( input_file_name, 'r' )
  q = h5_file['q'][:]
  v = h5_file['v'][:]
  pln_nrmls = h5_file['static_plane_normals'][:]
  iteration = h5_file['iteration'][0,0]
  timestep = h5_file['timestep'][0,0]
  h5_file.close()
except:
  print 'Failed to read in HDF5 file, exiting.'
  sys.exit(1)

time = iteration * timestep

# Compute the angle of the plane
theta = abs( math.atan2( pln_nrmls[0,0], pln_nrmls[1,0] ) )
# Compute the analytical position of the particle
dplane = 0.5 * time * time * ( -10.0 * math.sin(theta) + 5.0 * math.cos(theta) )
dx =  dplane * math.cos(theta)
if pln_nrmls[0,0] > 0 : 
  dx *= -1.0
dy =  dplane * math.sin(theta)
# Compute the analytical velocity of the partcile
vplane = time * ( -10.0 * math.sin(theta) + 5.0 * math.cos(theta) )
vx =  vplane * math.cos(theta)
if pln_nrmls[0,0] > 0 : 
  vx *= -1.0
vy =  vplane * math.sin(theta)

# Can tighten these tolerances further by decreasing the time step, but then the simulations are slow
if abs( q[0] - dx ) > 5.1e-5 :
  print "First q component incorect"
  sys.exit(1)
if abs( q[1] - dy ) > 0.00025 :
  print "Second q component incorect"
  sys.exit(1)
if abs( vx - v[0] ) > 2.03e-05 : 
  print "First v component incorrect"
  sys.exit(1)
if abs( vy - v[1] ) > 9.62e-05 : 
  print "Second v component incorrect"
  sys.exit(1)

sys.exit(0)
