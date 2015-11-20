// StaticPlane.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "StaticPlane.h"

#include "scisim/Math/MathUtilities.h"

StaticPlane::StaticPlane( const Vector2s& x, const Vector2s& n )
: m_x( x )
, m_v( 0.0, 0.0 )
, m_n( n.normalized() )
, m_t( -m_n.y(), m_n.x() ) // This choice of rotation ensures a right handed coordinate system if stacked [ n t ] in a matrix
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( m_t ) ) <= 1.0e-6 );
  // TODO: Check determinant == 1
}

StaticPlane::StaticPlane( std::istream& input_stream )
: m_x( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_v( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_n( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_t( MathUtilities::deserialize<Vector2s>( input_stream ) )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( m_t ) ) <= 1.0e-6 );
  // TODO: Check determinant == 1
}

StaticPlane::StaticPlane()
: m_x()
, m_v()
, m_n()
, m_t()
{}

Vector2s& StaticPlane::x()
{
  return m_x;
}

const Vector2s& StaticPlane::x() const
{
  return m_x;
}

Vector2s& StaticPlane::v()
{
  return m_v;
}

const Vector2s& StaticPlane::v() const
{
  return m_v;
}

const Vector2s& StaticPlane::n() const
{
  return m_n;
}

const Vector2s& StaticPlane::t() const
{
  return m_t;
}

bool StaticPlane::distanceLessThanZero( const Vector2s& x ) const
{
  return m_n.dot( x - m_x ) < 0.0;
}

bool StaticPlane::distanceLessThanOrEqualZero( const Vector2s& x, const scalar& r ) const
{
  return m_n.dot( x - m_x ) <= r;
}

void StaticPlane::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_x, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_n, output_stream );
  MathUtilities::serialize( m_t, output_stream );
}
