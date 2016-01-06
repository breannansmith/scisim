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

static Matrix22sr absR( const scalar& theta )
{
  Matrix22sr Q;
  Q(0,0) = fabs( cos( theta ) );
  Q(1,0) = fabs( sin( theta ) );
  Q(1,1) = Q(0,0);
  Q(0,1) = Q(1,0);
  return Q;
}

void BoxGeometry::AABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const
{
  const Matrix22sr Q{ absR( theta ) };
  const Array2s extents{ Q * m_r };
  min = x.array() - extents;
  max = x.array() + extents;
  std::cout << "BBOX: " << min.transpose() << "  ->  " << max.transpose() << std::endl;
}

void BoxGeometry::massAndInertia( const scalar& density, scalar& m, scalar& I ) const
{
  m = density * 4.0 * m_r.x() * m_r.y();
  std::cout << "m: " << m << std::endl;
  I = m * m_r.squaredNorm() / 3.0;
  std::cout << "I: " << I << std::endl;
}

void BoxGeometry::serializeState( std::ostream& output_stream ) const
{
  MathUtilities::serialize( m_r, output_stream );
}

const Vector2s& BoxGeometry::r() const
{
  return m_r;
}
