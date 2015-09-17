// StaticPlane.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "StaticPlane.h"

#include "scisim/Math/MathUtilities.h"

StaticPlane::StaticPlane()
: m_x()
, m_R()
, m_v()
, m_omega()
{}

StaticPlane::StaticPlane( const Vector3s& x, const Vector3s& n )
: m_x( x )
, m_R( Quaternions::FromTwoVectors( Vector3s::UnitY(), n.normalized() ) )
, m_v( Vector3s::Zero() )
, m_omega( Vector3s::Zero() )
{
  assert( fabs ( m_R.norm() - 1.0 ) < 1.0e-6 );
}

StaticPlane::StaticPlane( std::istream& input_stream )
: m_x( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_R( MathUtilities::deserializeQuaternion<scalar>( input_stream ) )
, m_v( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_omega( MathUtilities::deserialize<Vector3s>( input_stream ) )
{
  assert( fabs ( m_R.norm() - 1.0 ) < 1.0e-6 );
}

scalar StaticPlane::distanceToPoint( const Vector3s& x ) const
{
  return n().dot( x - this->x() );
}

void StaticPlane::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_R, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_omega, output_stream );
}

Vector3s& StaticPlane::x()
{
  return m_x;
}

const Vector3s& StaticPlane::x() const
{
  return m_x;
}

Quaternions& StaticPlane::R()
{
  return m_R;
}

const Quaternions& StaticPlane::R() const
{
  return m_R;
}

const Vector3s StaticPlane::n() const
{
  return m_R * Vector3s::UnitY();
}

const Vector3s StaticPlane::t0() const
{
  return m_R * Vector3s::UnitX();
}

const Vector3s StaticPlane::t1() const
{
  return m_R * Vector3s::UnitZ();
}

Vector3s& StaticPlane::v()
{
  return m_v;
}

const Vector3s& StaticPlane::v() const
{
  return m_v;
}

Vector3s& StaticPlane::omega()
{
  return m_omega;
}

const Vector3s& StaticPlane::omega() const
{
  return m_omega;
}
