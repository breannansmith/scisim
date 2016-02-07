// RigidBody2DStaticPlane.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "RigidBody2DStaticPlane.h"

#include "scisim/Math/MathUtilities.h"

RigidBody2DStaticPlane::RigidBody2DStaticPlane( const Vector2s& x, const Vector2s& n )
: m_x( x )
, m_n( n )
, m_t( -n.y(), n.x() )
, m_v( Vector2s::Zero() )
, m_omega( 0.0 )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( m_t ) ) <= 1.0e-6 );
  // TODO: Check determinant == 1
}

RigidBody2DStaticPlane::RigidBody2DStaticPlane( std::istream& input_stream )
: m_x( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_n( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_t( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_v( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_omega( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( m_t ) ) <= 1.0e-6 );
  // TODO: Check determinant == 1
}

const Vector2s& RigidBody2DStaticPlane::x() const
{
  return m_x;
}

const Vector2s& RigidBody2DStaticPlane::n() const
{
  return m_n;
}

const Vector2s& RigidBody2DStaticPlane::t() const
{
  return m_t;
}

const Vector2s& RigidBody2DStaticPlane::v() const
{
  return m_v;
}

const scalar& RigidBody2DStaticPlane::omega() const
{
  return m_omega;
}

void RigidBody2DStaticPlane::setX( const Vector2s& x )
{
  m_x = x;
}

void RigidBody2DStaticPlane::setN( const Vector2s& n )
{
  m_n = n.normalized();
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-9 );
  m_t << -m_n.y(), m_n.x();
}

void RigidBody2DStaticPlane::setV( const Vector2s& v )
{
  m_v = v;
}

void RigidBody2DStaticPlane::setOmega( const scalar& omega )
{
  m_omega = omega;
}

bool RigidBody2DStaticPlane::distanceLessThanZero( const Vector2s& x ) const
{
  return m_n.dot( x - m_x ) < 0.0;
}

scalar RigidBody2DStaticPlane::distanceToPoint( const Vector2s& x ) const
{
  return m_n.dot( x - m_x );
}

void RigidBody2DStaticPlane::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_n, output_stream );
  MathUtilities::serialize( m_t, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  Utilities::serializeBuiltInType( m_omega, output_stream );
}
