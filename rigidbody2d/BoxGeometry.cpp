// BoxGeometry.cpp
//
// Breannan Smith
// Last updated: 01/05/2016

#include "BoxGeometry.h"

#include "scisim/Math/MathUtilities.h"

BoxGeometry::BoxGeometry( const Vector2s& r )
: m_r( r )
{
  assert( ( m_r.array() > 0.0 ).all() );
}

BoxGeometry::BoxGeometry( std::istream& input_stream )
: m_r( MathUtilities::deserialize<Vector2s>( input_stream ) )
{
  assert( ( m_r.array() > 0.0 ).all() );
}

RigidBody2DGeometryType BoxGeometry::type() const
{
  return RigidBody2DGeometryType::BOX;
}

std::unique_ptr<RigidBody2DGeometry> BoxGeometry::clone() const
{
  return std::unique_ptr<RigidBody2DGeometry>{ new BoxGeometry{ m_r } };
}

// TMP
#include <iostream>

void BoxGeometry::AABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const
{
  std::cerr << "BoxGeometry::AABB" << std::endl;
  std::exit( EXIT_FAILURE );
  //min = x.array() - m_r;
  //max = x.array() + m_r;
}

void BoxGeometry::massAndInertia( const scalar& density, scalar& m, scalar& I ) const
{
  std::cerr << "BoxGeometry::massAndInertia" << std::endl;
  std::exit( EXIT_FAILURE );
  //m = density * MathDefines::PI<scalar>() * m_r * m_r;
  //I = 0.5 * m * m_r * m_r;
}

void BoxGeometry::serializeState( std::ostream& output_stream ) const
{
  MathUtilities::serialize( m_r, output_stream );
}
