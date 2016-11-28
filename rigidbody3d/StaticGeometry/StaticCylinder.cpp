// StaticCylinder.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "StaticCylinder.h"

#include "scisim/Utilities.h"
#include "scisim/Math/MathUtilities.h"

StaticCylinder::StaticCylinder( const Vector3s& x, const Vector3s& axis, const scalar& radius )
: m_x( x )
, m_n( axis.normalized() )
, m_theta( 0.0 )
, m_v( Vector3s::Zero() )
, m_omega( Vector3s::Zero() )
, m_r( radius )
{
  assert( fabs ( m_n.norm() - 1.0 ) < 1.0e-6 );
  assert( m_r > 0.0 );
}

StaticCylinder::StaticCylinder( std::istream& input_stream )
: m_x()
, m_n()
, m_theta()
, m_v()
, m_omega()
, m_r()
{
  // TODO: deserialize directly into the members above
  deserialize( input_stream );
}

void StaticCylinder::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_n, output_stream );
  Utilities::serialize( m_theta, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_omega, output_stream );
  Utilities::serialize( m_r, output_stream );
}

void StaticCylinder::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_x = MathUtilities::deserialize<Vector3s>( input_stream );
  m_n = MathUtilities::deserialize<Vector3s>( input_stream );
  m_theta = Utilities::deserialize<scalar>( input_stream );
  m_v = MathUtilities::deserialize<Vector3s>( input_stream );
  m_omega = MathUtilities::deserialize<Vector3s>( input_stream );
  m_r = Utilities::deserialize<scalar>( input_stream );
}

const Vector3s& StaticCylinder::x() const
{
  return m_x;
}

void StaticCylinder::setOrientation( const Vector3s& axis, const scalar& theta )
{
  m_n = axis.normalized();
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  m_theta = theta;
}

Quaternions StaticCylinder::R() const
{
  return AnglesAxis3s{ m_theta, m_n } * Quaternions::FromTwoVectors( Vector3s::UnitY(), m_n );
}

const Vector3s& StaticCylinder::v() const
{
  return m_v;
}

Vector3s& StaticCylinder::omega()
{
  return m_omega;
}

const Vector3s& StaticCylinder::omega() const
{
  return m_omega;
}

const scalar& StaticCylinder::r() const
{
  return m_r;
}

const Vector3s& StaticCylinder::axis() const
{
  return m_n;
}
