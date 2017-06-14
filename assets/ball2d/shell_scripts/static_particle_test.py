'''Checks whether a simulation that should be static is static.'''

import os
import sys
import argparse
import h5py

parser = argparse.ArgumentParser(description='Checks whether a simulation that should be static is static.')
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
except IOError as io_exception:
    sys.exit('HDF5 IO Error: ' + io_exception.message)
except KeyError as key_exception:
    sys.exit('HDF5 Key Error: ' + key_exception.message)

succeeded = True
if abs(q[0]) > args.t[0]:
    print 'q[0] residual', q[0]
    print 'First q component incorect'
    succeeded = False
if abs(q[1]) > args.t[1]:
    print 'q[1] residual', q[1]
    print 'Second q component incorect'
    succeeded = False
if abs(v[0]) > args.t[2]:
    print 'v[0] residual', v[0]
    print 'First v component incorrect'
    succeeded = False
if abs(v[1]) > args.t[3]:
    print 'v[1] residual', v[1]
    print 'Second v component incorrect'
    succeeded = False

if not succeeded:
    sys.exit(1)

sys.exit(0)
