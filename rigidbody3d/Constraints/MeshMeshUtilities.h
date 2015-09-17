// MeshMeshUtilities.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef MESH_MESH_UTILITIES_H
#define MESH_MESH_UTILITIES_H

#include "scisim/Math/MathDefines.h"

class RigidBodyTriangleMesh;

namespace MeshMeshUtilities
{

void computeActiveSet( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyTriangleMesh& mesh0,
                       const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyTriangleMesh& mesh1,
                       std::vector<Vector3s>& p, std::vector<Vector3s>& n );

// Returns a list of vertices that intersect the given half plane
void computeMeshHalfPlaneActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyTriangleMesh& mesh,
                                    const Vector3s& x0, const Vector3s& n, std::vector<unsigned>& vertices );

// Returns a list of vertices that intersect the given cylinder
void computeMeshCylinderActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyTriangleMesh& mesh,
                                   const Vector3s& x0, const Vector3s& axis, const scalar& r, std::vector<unsigned>& vertices );

}

#endif
