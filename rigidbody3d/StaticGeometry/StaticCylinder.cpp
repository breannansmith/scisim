// StaticCylinder.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "StaticCylinder.h"

#include "scisim/Utilities.h"
#include "scisim/Math/MathUtilities.h"

StaticCylinder::StaticCylinder()
: m_x()
, m_R()
, m_v()
, m_omega()
, m_r()
{}

StaticCylinder::StaticCylinder( const Vector3s& x, const Vector3s& axis, const scalar& radius )
: m_x( x )
, m_R( Quaternions::FromTwoVectors( Vector3s::UnitY(), axis.normalized() ) )
, m_v( Vector3s::Zero() )
, m_omega( Vector3s::Zero() )
, m_r( radius )
{
  assert( fabs ( m_R.norm() - 1.0 ) < 1.0e-6 );
  assert( m_r > 0.0 );
}

StaticCylinder::StaticCylinder( std::istream& input_stream )
: m_x()
, m_R()
, m_v()
, m_omega()
, m_r()
{
  deserialize( input_stream );
}

void StaticCylinder::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_R, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_omega, output_stream );
  Utilities::serializeBuiltInType( m_r, output_stream );
}

void StaticCylinder::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_x = MathUtilities::deserialize<Vector3s>( input_stream );
  MathUtilities::deserialize( m_R, input_stream );
  m_v = MathUtilities::deserialize<Vector3s>( input_stream );
  m_omega = MathUtilities::deserialize<Vector3s>( input_stream );
  m_r = Utilities::deserialize<scalar>( input_stream );
}

const Vector3s& StaticCylinder::x() const
{
  return m_x;
}

Quaternions& StaticCylinder::R()
{
  return m_R;
}

const Quaternions& StaticCylinder::R() const
{
  return m_R;
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

const Vector3s StaticCylinder::axis() const
{
  return m_R * Vector3s::UnitY();
}
