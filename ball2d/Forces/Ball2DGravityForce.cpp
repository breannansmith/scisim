// Ball2DForce.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "Ball2DGravityForce.h"

#include "SCISim/Math/MathUtilities.h"
#include "SCISim/StringUtilities.h"

Ball2DGravityForce::Ball2DGravityForce( const Vector2s& g )
: m_g( g )
{}

Ball2DGravityForce::Ball2DGravityForce( std::istream& input_stream )
: m_g( MathUtilities::deserialize<Vector2s>( input_stream ) )
{}

Ball2DGravityForce::~Ball2DGravityForce()
{}

scalar Ball2DGravityForce::computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const
{
  assert( q.size() % 2 == 0 ); assert( q.size() == M.rows() ); assert( q.size() == M.cols() );

  scalar U{ 0.0 };
  for( int i = 0; i < q.size(); i += 2 )
  {
    assert( M.valuePtr()[ i ] == M.valuePtr()[ i + 1 ] ); assert( M.valuePtr()[ i ] > 0.0 );
    U += - M.valuePtr()[ i ] * m_g.dot( q.segment<2>( i ) );
  }

  return U;
}

void Ball2DGravityForce::computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const
{
  assert( q.size() % 2 == 0 ); assert( q.size() == v.size() ); assert( q.size() == M.rows() );
  assert( q.size() == M.cols() ); assert( q.size() == result.size() );

  for( int i = 0; i < q.size(); i += 2 )
  {
    assert( M.valuePtr()[ i ] == M.valuePtr()[ i + 1 ] ); assert( M.valuePtr()[ i ] > 0.0 );
    result.segment<2>( i ) += M.valuePtr()[ i ] * m_g;
  }
}

std::unique_ptr<Ball2DForce> Ball2DGravityForce::clone() const
{
  return std::unique_ptr<Ball2DForce>{ new Ball2DGravityForce{ m_g } };
}

// TODO: Have a class enum to idetify the forces in lieu of strings
void Ball2DGravityForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serializeString( "ball2d_gravity_force", output_stream );
  MathUtilities::serialize( m_g, output_stream );
}
