#include "PenaltyForce.h"

#include "scisim/StringUtilities.h"
#include "scisim/Utilities.h"

PenaltyForce::PenaltyForce( const scalar& k, const scalar& power )
: m_k( k )
, m_power( power )
{
  assert( m_k > 0.0 );
}

PenaltyForce::PenaltyForce( std::istream& input_stream )
: m_k( Utilities::deserialize<scalar>( input_stream ) )
, m_power( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_k > 0.0 );
}

scalar PenaltyForce::computePotential( const VectorXs& q, const SparseMatrixsc& /*M*/, const VectorXs& r ) const
{
  assert( q.size() % 2 == 0 );
  assert( r.size() == q.size() / 2 );

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
      // U = 0.5 * k * pen_depth ^ power
      U += 0.5 * m_k * std::pow( -delta, m_power );
    }
  }

  return U;
}

void PenaltyForce::computeForce( const VectorXs& q, const VectorXs& /*v*/, const SparseMatrixsc& /*M*/, const VectorXs& r, VectorXs& result ) const
{
  assert( q.size() % 2 == 0 );
  assert( r.size() == q.size() / 2 );
  assert( q.size() == result.size() );

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
      // F = 0.5 * k * power * pen_depth ^ ( power - 1.0 )
      const Vector2s F{ 0.5 * m_k * m_power * std::pow( -d, m_power - 1.0 ) * n };
      result.segment<2>( 2 * ball1 ) += F;
      result.segment<2>( 2 * ball0 ) -= F;
    }
  }
}

std::unique_ptr<Ball2DForce> PenaltyForce::clone() const
{
  return std::unique_ptr<Ball2DForce>{ new PenaltyForce{ m_k, m_power } };
}

void PenaltyForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serialize( "hertzian_penalty", output_stream );
  Utilities::serialize( m_k, output_stream );
  Utilities::serialize( m_power, output_stream );
}
