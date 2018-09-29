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

void BoxGeometry::computeCollisionAABB( const Vector2s& /*x0*/, const scalar& /*theta0*/, const Vector2s& x1, const scalar& theta1, Array2s& min, Array2s& max ) const
{
  this->computeAABB( x1, theta1, min, max );
}

void BoxGeometry::computeAABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const
{
  const Array2s extents{ Eigen::Rotation2D<scalar>( theta ).matrix().cwiseAbs() * m_r };
  min = x.array() - extents;
  max = x.array() + extents;
}

void BoxGeometry::computeMassAndInertia( const scalar& density, scalar& m, scalar& I ) const
{
  m = density * 4.0 * m_r.x() * m_r.y();
  I = m * m_r.squaredNorm() / 3.0;
}

void BoxGeometry::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( RigidBody2DGeometryType::BOX, output_stream );
  MathUtilities::serialize( m_r, output_stream );
}

const Vector2s& BoxGeometry::r() const
{
  return m_r;
}
