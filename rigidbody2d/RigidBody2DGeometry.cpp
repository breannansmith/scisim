// RigidBody2DGeometry.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "RigidBody2DGeometry.h"

RigidBody2DGeometry::~RigidBody2DGeometry()
{}

void RigidBody2DGeometry::computeAABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const
{
  AABB( x, theta, min, max );
  assert( ( min < max ).all() );
}

void RigidBody2DGeometry::computeMassAndInertia( const scalar& density, scalar& m, scalar& I ) const
{
  assert( density > 0.0 );
  massAndInertia( density, m, I );
  assert( m > 0.0 );
  assert( I > 0.0 );
}

void RigidBody2DGeometry::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  serializeState( output_stream );
}
