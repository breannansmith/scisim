// HertzianPenaltyForce.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "HertzianPenaltyForce.h"

#include "scisim/StringUtilities.h"
#include "scisim/Utilities.h"

HertzianPenaltyForce::HertzianPenaltyForce( const scalar& k )
: m_k( k )
{
  assert( m_k > 0.0 );
}

HertzianPenaltyForce::HertzianPenaltyForce( std::istream& input_stream )
: m_k( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_k > 0.0 );
}

HertzianPenaltyForce::~HertzianPenaltyForce()
{}

scalar HertzianPenaltyForce::computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const
{
  assert( q.size() % 2 == 0 ); assert( q.size() == M.rows() ); assert( q.size() == M.cols() ); assert( r.size() == q.size() / 2 );

  scalar U{ 0.0 };
  // For each ball
  for( unsigned ball0 = 0; ball0 < r.size(); ++ball0 )
  {
    // For each subsequent ball
    for( unsigned ball1 = ball0 + 1; ball1 < r.size(); ++ball1 )
    {
      // Compute the total radius
      const scalar total_radius{ r(ball0) + r(ball1) };
      // Compute a vector pointing from ball0 to ball1
      const Vector2s n{ q.segment<2>( 2 * ball1 ) - q.segment<2>( 2 * ball0 ) };
      // If the squared distance is greater or equal to the sum of the radii squared, no force
      if( n.squaredNorm() > total_radius * total_radius )
      {
        continue;
      }
      // Compute the penetration depth
      const scalar delta{ n.norm() - total_radius };
      assert( delta < 0.0 );
      // U = 0.5 * k * pen_depth^(5/2)
      U += 0.5 * m_k * std::pow( -delta, scalar( 2.5 ) );
    }
  }

  return U;
}

void HertzianPenaltyForce::computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const
{
  assert( q.size() % 2 == 0 ); assert( q.size() == v.size() ); assert( q.size() == M.rows() );
  assert( q.size() == M.cols() ); assert( r.size() == q.size() / 2 ); assert( q.size() == result.size() );

  // For each ball
  for( unsigned ball0 = 0; ball0 < r.size(); ++ball0 )
  {
    // For each subsequent ball
    for( unsigned ball1 = ball0 + 1; ball1 < r.size(); ++ball1 )
    {
      // Compute the total radius
      const scalar total_radius{ r(ball0) + r(ball1) };
      // Compute a vector pointing from ball0 to ball1
      Vector2s n{ q.segment<2>( 2 * ball1 ) - q.segment<2>( 2 * ball0 ) };
      // Compute the squared length of the vector
      scalar d{ n.squaredNorm() };
      // If the squared distance is greater or equal to the sum of the radii squared, no force
      if( d > total_radius * total_radius )
      {
        continue;
      }
      // Normalize the vector between the balls
      d = sqrt( d );
      assert( d != 0.0 );
      n /= d;
      assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
      // Compute the penetration depth
      d -= total_radius;
      assert( d < 0.0 );
      // F = 5 * k * pen_depth^(3/2) * n
      const Vector2s F{ ( 5.0 / 4.0 ) * m_k * std::pow( -d, scalar( 1.5 ) ) * n };
      result.segment<2>( 2 * ball1 ) += F;
      result.segment<2>( 2 * ball0 ) -= F;
    }
  }
}

std::unique_ptr<Ball2DForce> HertzianPenaltyForce::clone() const
{
  return std::unique_ptr<Ball2DForce>{ new HertzianPenaltyForce{ m_k } };
}

void HertzianPenaltyForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serializeString( "hertzian_penalty", output_stream );
  Utilities::serializeBuiltInType( m_k, output_stream );
}
