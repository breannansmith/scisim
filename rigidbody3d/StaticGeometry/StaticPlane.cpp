// StaticPlane.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "StaticPlane.h"

#include "scisim/Math/MathUtilities.h"

StaticPlane::StaticPlane( const Vector3s& x, const Vector3s& n )
: m_x( x )
, m_n( n.normalized() )
, m_v( Vector3s::Zero() )
, m_omega( Vector3s::Zero() )
{}

StaticPlane::StaticPlane( std::istream& input_stream )
: m_x( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_n( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_v( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_omega( MathUtilities::deserialize<Vector3s>( input_stream ) )
{}

scalar StaticPlane::distanceToPoint( const Vector3s& x ) const
{
  return n().dot( x - this->x() );
}

void StaticPlane::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_n, output_stream );
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

Quaternions StaticPlane::R() const
{
  return Quaternions::FromTwoVectors( Vector3s::UnitY(), m_n );
}

const Vector3s StaticPlane::n() const
{
  return m_n;
}

const Vector3s StaticPlane::t0() const
{
  return Quaternions::FromTwoVectors( Vector3s::UnitY(), m_n ) * Vector3s::UnitX();
}

const Vector3s StaticPlane::t1() const
{
  return Quaternions::FromTwoVectors( Vector3s::UnitY(), m_n ) * Vector3s::UnitZ();
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
