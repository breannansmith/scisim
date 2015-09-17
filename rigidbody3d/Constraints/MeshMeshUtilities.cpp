// MeshMeshUtilities.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "MeshMeshUtilities.h"

#include "rigidbody3d/Geometry/RigidBodyTriangleMesh.h"

void MeshMeshUtilities::computeActiveSet( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyTriangleMesh& mesh0,
                                          const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyTriangleMesh& mesh1,
                                          std::vector<Vector3s>& p, std::vector<Vector3s>& n )
{
  // mesh0 against mesh1
  {
    const Matrix3Xsc& samples0{ mesh0.samples() };
    // Transformation to take points in mesh0 to mesh1
    const Matrix3s R01{ R1.transpose() * R0 };
    const Vector3s x01{ R1.transpose() * ( cm0 - cm1 ) };
    // For each surface sample of mesh0
    for( int smp_num = 0; smp_num < samples0.cols(); ++smp_num )
    {
      // Transform mesh0's sample into mesh1's frame
      const Vector3s x{ R01 * samples0.col( smp_num ) + x01 };
      // Determine if a collision occurs
      Vector3s normal;
      const bool collision_occurs{ mesh1.detectCollision( x, normal ) };
      // Cache out the world space collision point and normal if a collision happens
      if( collision_occurs )
      {
        assert( fabs( normal.norm() - 1.0 ) <= 1.0e-6 );
        assert( ( R1 * x + cm1 - R0 * samples0.col( smp_num ) - cm0 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
        // Transform the contact point and normal from mesh1's frame to world space
        p.emplace_back( R1 * x + cm1 );
        n.emplace_back( R1 * normal );
      }
    }
  }

  // mesh1 against mesh0
  {
    const Matrix3Xsc& samples1{ mesh1.samples() };
    // Transformation to take points in mesh1 to mesh0
    const Matrix3s R10{ R0.transpose() * R1 };
    const Vector3s x10{ R0.transpose() * ( cm1 - cm0 ) };
    // For each surface sample of mesh1
    for( int smp_num = 0; smp_num < samples1.cols(); ++smp_num )
    {
      // Transform mesh1's sample into mesh0's frame
      const Vector3s x{ R10 * samples1.col( smp_num ) + x10 };
      // Determine if a collision occurs
      Vector3s normal;
      const bool collision_occurs{ mesh0.detectCollision( x, normal ) };
      // Cache out the world space collision point and normal if a collision happens
      if( collision_occurs )
      {
        assert( fabs( normal.norm() - 1.0 ) <= 1.0e-6 );
        assert( ( R0 * x + cm0 - R1 * samples1.col( smp_num ) - cm1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
        // Transform the contact point and normal back to world space
        p.emplace_back( R0 * x + cm0 );
        n.emplace_back( - R0 * normal );
      }
    }
  }
}

void MeshMeshUtilities::computeMeshHalfPlaneActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyTriangleMesh& mesh,
                                                       const Vector3s& x0, const Vector3s& n, std::vector<unsigned>& vertices )
{
  assert( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( R.determinant() - 1.0 ) <= 1.0e-6 );

  const Matrix3Xsc& convex_hull_verts{ mesh.convexHullVertices() };

  // For each vertex of the mesh
  for( unsigned vrt_idx = 0; vrt_idx < unsigned( convex_hull_verts.cols() ); ++vrt_idx )
  {
    const Vector3s v{ R * convex_hull_verts.col( vrt_idx ) + cm };
    // Compute the distance from the vertex to the halfplane
    const scalar d{ n.dot( v - x0 ) };
    // If the distance is negative
    if( d <= 0.0 )
    {
      vertices.emplace_back( vrt_idx );
    }
  }
}

void MeshMeshUtilities::computeMeshCylinderActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyTriangleMesh& mesh, const Vector3s& x0, const Vector3s& axis, const scalar& r, std::vector<unsigned>& vertices )
{
  assert( fabs( axis.norm() - 1.0 ) <= 1.0e-6 );
  assert( r > 0.0 );

  const Matrix3Xsc& convex_hull_verts{ mesh.convexHullVertices() };

  // For each vertex of the mesh
  for( unsigned vrt_idx = 0; vrt_idx < unsigned( convex_hull_verts.cols() ); ++vrt_idx )
  {
    // Compute the transformed position of the vertex
    const Vector3s v{ R * convex_hull_verts.col( vrt_idx ) + cm };
    // Vector along the horizontal extent of the cylinder
    const Vector3s d{ v - x0 - axis.dot( v - x0 ) * axis };
    assert( fabs( d.dot( axis ) ) <= 1.0e-6 );
    if( d.squaredNorm() >= r * r )
    {
      vertices.emplace_back( vrt_idx );
    }
  }
}
