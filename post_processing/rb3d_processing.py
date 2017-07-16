'''Tools for loading 3D rigid body simulations.'''

import sys
import numpy


def matrixIsRotation(R):
    '''Check if a 3x3 matrix is orthonormal and orientation preserving.'''
    assert R.shape == (3, 3)
    idresid = numpy.amax(numpy.absolute(numpy.transpose(numpy.matrix(R)) * numpy.matrix(R) - numpy.identity(3)))
    if idresid > 1.0e-9:
        return False
    if abs(numpy.linalg.det(R) - 1.0) > 1.0e-9:
        return False
    return True


class DiscreteState(object):
    '''A container for the state of a 3D rigid body simulation.'''
    def __init__(self, h5_file):
        try:
            self.q = h5_file['/state/q'][:, 0]
            assert isinstance(self.q, numpy.ndarray)
            assert self.q.ndim == 1
            assert self.q.shape[0] % 12 == 0
            self.nbodies = self.q.shape[0] / 12
            self.v = h5_file['/state/v'][:, 0]
            assert isinstance(self.v, numpy.ndarray)
            assert self.v.ndim == 1
            assert self.v.shape[0] == 6 * self.nbodies
            self.M0 = h5_file['/state/M0'][:, 0]
            assert isinstance(self.M0, numpy.ndarray)
            assert self.M0.ndim == 1
            assert self.M0.shape[0] == 6 * self.nbodies
            self.geo_indices = h5_file['/geometry/geometry_indices'][:, :]
            assert isinstance(self.geo_indices, numpy.ndarray)
            assert self.geo_indices.ndim == 2
            assert self.geo_indices.shape == (2, self.nbodies)
            self.kinematically_scripted = h5_file['/state/kinematically_scripted'][:, 0]
            assert isinstance(self.kinematically_scripted, numpy.ndarray)
            assert self.kinematically_scripted.ndim == 1
            assert self.kinematically_scripted.shape[0] == self.nbodies
            if '/geometry/spheres' in h5_file:
                self.spheres = h5_file['/geometry/spheres'][:]
            else:
                self.spheres = numpy.empty([0])
            if '/geometry/meshes' in h5_file:
                self.meshes = h5_file['/geometry/meshes'][:]
            else:
                self.meshes = numpy.empty([0])
            self.git_hash = h5_file['git_hash'][:][0]
            self.iteration = h5_file['iteration'][:][0, 0]
            assert self.iteration >= 0
            self.time = h5_file['time'][:][0, 0]
            assert self.time >= 0
            self.timestep = h5_file['timestep'][:][0, 0]
            assert self.timestep >= 0.0
            # ... there is more data in the file, I'm just not reading it in here. h5ls -r file_name from the command line will show all data paths.
        except KeyError as key_exception:
            sys.exit('HDF5 Key Error: ' + key_exception.message)

    # def sphereBodies(self):
    #     '''Iterates over the sphere geometry in the system.'''
    #     bdy_idx = 0
    #     while bdy_idx < self.nbodies:
    #         if self.geo_indices[0, bdy_idx] == 1:
    #             geo_idx = self.geo_indices[1, bdy_idx]
    #             assert geo_idx >= 0 and geo_idx < len(self.spheres)
    #             radius = self.spheres[geo_idx]['r']
    #             assert radius > 0.0
    #             kinematic = self.kinematically_scripted[bdy_idx]
    #             x = self.q[3 * bdy_idx: 3 * bdy_idx + 3]
    #             R = numpy.zeros((3, 3))
    #             R[0, :] = self.q[3 * self.nbodies + 9 * bdy_idx: 3 * self.nbodies + 9 * bdy_idx + 3]
    #             R[1, :] = self.q[3 * self.nbodies + 9 * bdy_idx + 3: 3 * self.nbodies + 9 * bdy_idx + 6]
    #             R[2, :] = self.q[3 * self.nbodies + 9 * bdy_idx + 6: 3 * self.nbodies + 9 * bdy_idx + 9]
    #             assert matrixIsRotation(R)
    #             yield x, R, radius, kinematic
    #         bdy_idx += 1

    def meshBodies(self):
        '''Iterates over the mesh geometry in the system.'''
        bdy_idx = 0
        while bdy_idx < self.nbodies:
            if self.geo_indices[0, bdy_idx] == 3:
                geo_idx = self.geo_indices[1, bdy_idx]
                assert geo_idx >= 0 and geo_idx < len(self.meshes)
                mesh_name = self.meshes[geo_idx][0]
                kinematic = self.kinematically_scripted[bdy_idx]
                x = self.q[3 * bdy_idx: 3 * bdy_idx + 3]
                R = numpy.zeros((3, 3))
                R[0, :] = self.q[3 * self.nbodies + 9 * bdy_idx: 3 * self.nbodies + 9 * bdy_idx + 3]
                R[1, :] = self.q[3 * self.nbodies + 9 * bdy_idx + 3: 3 * self.nbodies + 9 * bdy_idx + 6]
                R[2, :] = self.q[3 * self.nbodies + 9 * bdy_idx + 6: 3 * self.nbodies + 9 * bdy_idx + 9]
                assert matrixIsRotation(R)
                vel = self.v[3 * bdy_idx: 3 * bdy_idx + 3]
                omega = self.v[3 * self.nbodies + 3 * bdy_idx: 3 * self.nbodies + 3 * bdy_idx + 3]
                M = self.M0[3 * bdy_idx]
                assert M > 0.0
                assert M == self.M0[3 * bdy_idx + 1]
                assert M == self.M0[3 * bdy_idx + 2]
                I0 = self.M0[3 * self.nbodies + 3 * bdy_idx: 3 * self.nbodies + 3 * bdy_idx + 3]
                assert numpy.all(I0 > 0.0)
                I = numpy.dot(numpy.dot(R, numpy.diag(I0)), numpy.transpose(R))
                yield x, R, vel, omega, M, I, kinematic, mesh_name
            bdy_idx += 1


class CollisionState(object):
    '''A container for collisions from a 3D rigid body simulation.'''
    def __init__(self, h5_file):
        try:
            self.count = h5_file['collision_count'][0, 0]
            assert self.count >= 0
            self.impulses = h5_file['collision_forces'][:]
            assert isinstance(self.impulses, numpy.ndarray)
            assert self.impulses.ndim == 2
            assert self.impulses.shape == (3, self.count)
            self.indices = h5_file['collision_indices'][:]
            assert isinstance(self.indices, numpy.ndarray)
            assert self.indices.ndim == 2
            assert self.indices.shape == (2, self.count)
            self.normals = h5_file['collision_normals'][:]
            assert isinstance(self.normals, numpy.ndarray)
            assert self.normals.ndim == 2
            assert self.normals.shape == (3, self.count)
            self.points = h5_file['collision_points'][:]
            assert isinstance(self.points, numpy.ndarray)
            assert self.points.ndim == 2
            assert self.points.shape == (3, self.count)
            self.git_hash = h5_file['git_hash'][:][0]
            self.iteration = h5_file['iteration'][:][0, 0]
            assert self.iteration >= 0
            self.time = h5_file['time'][:][0, 0]
            assert self.time >= 0
            self.timestep = h5_file['timestep'][:][0, 0]
            assert self.timestep >= 0.0
        except KeyError as key_exception:
            sys.exit('HDF5 Key Error: ' + key_exception.message)

    def collisions(self):
        '''Iterates over the collisions in the system.'''
        col_idx = 0
        while col_idx < self.count:
            impulse = self.impulses[:, col_idx]
            indices = self.indices[:, col_idx]
            normal = self.normals[:, col_idx]
            assert abs(numpy.linalg.norm(normal) - 1.0) <= 1.0e-6
            point = self.points[:, col_idx]
            yield impulse, indices, normal, point
            col_idx += 1
