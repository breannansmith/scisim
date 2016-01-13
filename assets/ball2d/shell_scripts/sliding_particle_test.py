''' Compares a simulation of a ball sliding down a plane to the analytical solution. '''

import os
import sys
import argparse
import math
import h5py

parser = argparse.ArgumentParser( description='Compares a simulation of a ball sliding down a plane to the analytical solution.' )
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
    pln_nrmls = h5_file['static_plane_normals'][:]
    iteration = h5_file['iteration'][0,0]
    timestep = h5_file['timestep'][0,0]
except IOError as io_exception:
  sys.exit('HDF5 IO Error: ' + io_exception.message)
except KeyError as key_exception:
  sys.exit('HDF5 Key Error: ' + key_exception.message)

time = iteration * timestep

# Note: Gravity and friction are hardcoded, for now
gravity_magnitude = 10.0
mu = 0.5

# Compute the angle of the plane
theta = abs( math.atan2( pln_nrmls[0,0], pln_nrmls[1,0] ) )
# Compute the analytical position of the ball
dplane = 0.5 * time * time * ( -gravity_magnitude * math.sin(theta) + gravity_magnitude * mu * math.cos(theta) )
dx = dplane * math.cos(theta)
if pln_nrmls[0,0] > 0:
  dx *= -1.0
dy = dplane * math.sin(theta)
# Compute the analytical velocity of the ball
vplane = time * ( -gravity_magnitude * math.sin(theta) + gravity_magnitude * mu * math.cos(theta) )
vx = vplane * math.cos(theta)
if pln_nrmls[0,0] > 0:
  vx *= -1.0
vy = vplane * math.sin(theta)

# Can tighten these tolerances further by decreasing the time step, but then the simulations are slow
if abs( q[0] - dx ) > 5.1e-5:
  sys.exit('First q component incorect')
if abs( q[1] - dy ) > 0.00025:
  sys.exit('Second q component incorect')
if abs( vx - v[0] ) > 2.03e-05:
  sys.exit('First v component incorrect')
if abs( vy - v[1] ) > 9.62e-05:
  sys.exit('Second v component incorrect')

sys.exit(0)
