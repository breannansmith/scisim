'''Compares a simulation of a ball sliding down a plane to the analytical solution.'''

import os
import sys
import argparse
import math
import h5py

parser = argparse.ArgumentParser(description='Compares a simulation of a ball sliding down a plane to the analytical solution.')
parser.add_argument('-i', metavar='input_hdf5_file', type=str, nargs=1, help='input HDF5 file name', required=True)
parser.add_argument('-t', metavar='tolerances', type=float, nargs=4, help='tolerances for x, y, vx, and vy', required=True)
args = parser.parse_args()

if not all([t >= 0.0 for t in args.t]):
    sys.exit('Error, all tolerances must be non-negative.')

input_file_name = args.i[0]
if not os.path.isfile(input_file_name):
    sys.exit('Error, input file \'' + input_file_name + '\' does not exist.')

print 'Validating:', input_file_name

try:
    with h5py.File(input_file_name, 'r') as h5_file:
        q = h5_file['q'][:]
        v = h5_file['v'][:]
        pln_nrmls = h5_file['static_plane_normals'][:]
        iteration = h5_file['iteration'][0, 0]
        timestep = h5_file['timestep'][0, 0]
except IOError as io_exception:
    sys.exit('HDF5 IO Error: ' + io_exception.message)
except KeyError as key_exception:
    sys.exit('HDF5 Key Error: ' + key_exception.message)

time = iteration * timestep

# Note: Gravity and friction are hardcoded, for now
gravity_magnitude = 10.0
mu = 0.5

# Compute the angle of the plane
theta = abs(math.atan2(pln_nrmls[0, 0], pln_nrmls[1, 0]))
# Compute the analytical position of the ball
dplane = 0.5 * time * time * (-gravity_magnitude * math.sin(theta) + gravity_magnitude * mu * math.cos(theta))
dx = dplane * math.cos(theta)
if pln_nrmls[0, 0] > 0:
    dx *= -1.0
dy = dplane * math.sin(theta)
# Compute the analytical velocity of the ball
vplane = time * (-gravity_magnitude * math.sin(theta) + gravity_magnitude * mu * math.cos(theta))
vx = vplane * math.cos(theta)
if pln_nrmls[0, 0] > 0:
    vx *= -1.0
vy = vplane * math.sin(theta)

succeeded = True
if abs(q[0] - dx) > args.t[0]:
    print 'q[0] residual', abs(q[0] - dx)
    print 'First q component incorect'
    succeeded = False
if abs(q[1] - dy) > args.t[1]:
    print 'q[1] residual', abs(q[1] - dy)
    print 'Second q component incorect'
    succeeded = False
if abs(vx - v[0]) > args.t[2]:
    print 'v[0] residual', abs(vx - v[0])
    print 'First v component incorrect'
    succeeded = False
if abs(vy - v[1]) > args.t[3]:
    print 'v[1] residual', abs(vy - v[1])
    print 'Second v component incorrect'
    succeeded = False

if not succeeded:
    sys.exit(1)

sys.exit(0)
