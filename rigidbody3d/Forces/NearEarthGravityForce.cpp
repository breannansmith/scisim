// NearEarthGravityForce.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "NearEarthGravityForce.h"

#include "scisim/StringUtilities.h"
#include "scisim/Math/MathUtilities.h"

NearEarthGravityForce::NearEarthGravityForce( const Vector3s& g )
: m_g( g )
{}

NearEarthGravityForce::NearEarthGravityForce( std::istream& input_stream )
: m_g( MathUtilities::deserialize<Vector3s>( input_stream ) )
{}

NearEarthGravityForce::~NearEarthGravityForce()
{}

scalar NearEarthGravityForce::computePotential( const VectorXs& q, const SparseMatrixsc& M ) const
{
  assert( q.size() % 12 == 0 ); assert( M.rows() == M.cols() ); assert( M.nonZeros() == q.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  const Eigen::Map<const VectorXs> masses{ M.valuePtr(), 3 * nbodies };

  scalar U = 0.0;
  for( unsigned i = 0; i < nbodies; ++i )
  {
    assert( masses( 3 * i + 0 ) == masses( 3 * i + 1 ) ); assert( masses( 3 * i + 1 ) == masses( 3 * i + 2 ) );
    U += - masses( 3 * i ) * m_g.dot( q.segment<3>( 3 * i ) );
  }
  return U;
}

void NearEarthGravityForce::computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const
{
  assert( q.size() % 12 == 0 ); assert( v.size() == q.size() / 2 );
  assert( M.rows() == M.cols() ); assert( M.nonZeros() == q.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  const Eigen::Map<const VectorXs> masses{ M.valuePtr(), 3 * nbodies };

  for( unsigned i = 0; i < nbodies; ++i )
  {
    assert( masses( 3 * i + 0 ) == masses( 3 * i + 1 ) ); assert( masses( 3 * i + 1 ) == masses( 3 * i + 2 ) );
    result.segment<3>( 3 * i ) += masses( 3 * i ) * m_g;
  }
}

std::string NearEarthGravityForce::name() const
{
  return "near_earth_gravity";
}

std::unique_ptr<Force> NearEarthGravityForce::clone() const
{
  return std::unique_ptr<Force>{ new NearEarthGravityForce{ m_g } };
}

void NearEarthGravityForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serialize( name(), output_stream );
  MathUtilities::serialize( m_g, output_stream );
}

void NearEarthGravityForce::setForce( const Vector3s& g )
{
  m_g = g;
}
