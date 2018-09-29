#include "NearEarthGravityForce.h"

#include "scisim/Math/MathUtilities.h"

NearEarthGravityForce::NearEarthGravityForce( const Vector2s& g )
: m_g( g )
{}

NearEarthGravityForce::NearEarthGravityForce( std::istream& input_stream )
: m_g( MathUtilities::deserialize<Vector2s>( input_stream ) )
{}

scalar NearEarthGravityForce::computePotential( const VectorXs& q, const SparseMatrixsc& M ) const
{
  assert( q.size() % 3 == 0 ); assert( M.rows() == M.cols() ); assert( M.nonZeros() == q.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 3 ) };

  const Eigen::Map<const VectorXs> masses{ M.valuePtr(), M.nonZeros() };

  scalar U = 0.0;
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    assert( masses( 3 * bdy_idx + 0 ) == masses( 3 * bdy_idx + 1 ) ); assert( masses( 3 * bdy_idx + 0 ) > 0.0 );
    U += - masses( 3 * bdy_idx ) * m_g.dot( q.segment<2>( 3 * bdy_idx ) );
  }
  return U;
}

void NearEarthGravityForce::computeForce( const VectorXs& q, const VectorXs& /*v*/, const SparseMatrixsc& M, VectorXs& result ) const
{
  assert( q.size() % 3 == 0 ); assert( M.rows() == M.cols() ); assert( M.nonZeros() == q.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 3 ) };

  const Eigen::Map<const VectorXs> masses{ M.valuePtr(), M.nonZeros() };

  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    assert( masses( 3 * bdy_idx + 0 ) == masses( 3 * bdy_idx + 1 ) ); assert( masses( 3 * bdy_idx + 0 ) > 0.0 );
    result.segment<2>( 3 * bdy_idx ) += masses( 3 * bdy_idx ) * m_g;
  }
}

std::unique_ptr<RigidBody2DForce> NearEarthGravityForce::clone() const
{
  return std::unique_ptr<NearEarthGravityForce>{ new NearEarthGravityForce{ m_g } };
}

void NearEarthGravityForce::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( RigidBody2DForceType::NEAR_EARTH_GRAVITY, output_stream );
  MathUtilities::serialize( m_g, output_stream );
}
