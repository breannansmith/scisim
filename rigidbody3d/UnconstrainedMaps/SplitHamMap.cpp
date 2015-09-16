// SplitHamMap.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "SplitHamMap.h"

#include "IntegrationUtils.h"

#include "SCISim/UnconstrainedMaps/FlowableSystem.h"

#ifndef NDEBUG
#include <iostream>
#endif

SplitHamMap::~SplitHamMap()
{}

void SplitHamMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  assert( iteration > 0 );
  const scalar next_time{ iteration * dt };

  assert( q0.size() % 12 == 0 );
  assert( q0.size() == 2 * v0.size() );
  assert( fsys.M().nonZeros()  == q0.size() );
  assert( fsys.M0().nonZeros() == v0.size() );
  // Make sure kinematic body's aren't expecting to get integrated
  #ifndef NDEBUG
  {
    const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };
    for( unsigned i = 0; i < nbodies; ++i )
    {
      if( fsys.isKinematicallyScripted( i ) )
      {
        assert( ( v0.segment<3>( 3 * i ).array() == 0.0 ).all() );
        assert( ( v0.segment<3>( 3 * nbodies + 3 * i ).array() == 0.0 ).all() );
      }
    }
  }
  #endif

  const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };

  q1 = q0;
  v1 = fsys.M() * v0; // A bit of a misnomer as this actually stores momentum for most of this function

  // Compute start force
  VectorXs F{ v0.size() };
  fsys.computeForce( q0, v0, next_time, F ); // Hamiltonian so there shouldn't be velocity dependent forces

  // First momentum update
  // p1 += 0.5 * h * F_q0;
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }
    v1.segment<3>( 3 * bdy_idx ) += 0.5 * dt * F.segment<3>( 3 * bdy_idx );
    v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) += 0.5 * dt * F.segment<3>( 3 * nbodies + 3 * bdy_idx );
  }

  // Linear position update
  const VectorXs q_update{ dt * v0 + 0.5 * dt * dt * fsys.Minv() * F };
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }
    q1.segment<3>( 3 * bdy_idx ) += q_update.segment<3>( 3 * bdy_idx );
  }

  // Grab vector of reference frame mass/inertia
  const SparseMatrixsc& M0{ fsys.M0() };
  const Eigen::Map<const VectorXs,Eigen::Aligned> M0_vals{ M0.valuePtr(), M0.nonZeros() };

  // Split Hamiltonian Update (Q,L) per body
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }

    // Grab a map to the input orientation matrix
    const Eigen::Map<const Matrix33sr> R0{ &q0.data()[ 3 * nbodies + 9 * bdy_idx ] };

    // Ensure we have an orthonormal rotation matrix
    assert( fabs( R0.determinant() - 1.0 ) <= 1.0e-9 );
    assert( ( R0 * R0.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );

    // Grab a map to the storage for the output orientation matrix
    Eigen::Map<Matrix33sr> R1{ &q1.data()[ 3 * nbodies + 9 * bdy_idx ] };

    // At this point, R0 and Q1 should be identical
    assert( ( R0 - R1 ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );

    // Get the body-frame diagonalized inertia tensor
    const Vector3s I{ M0_vals.segment<3>( 3 * nbodies + 3 * bdy_idx ) };
    assert( ( I.array() > 0.0 ).all() );

    // Get the body angular momentum
    Vector3s pAngB{ R0.transpose() * v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) };

    // Split integrate. Note that rotations are counterclockwise.
    AnglesAxis3s R;

    R.angle() = 0.5 * dt * pAngB.z() / I.z();
    R.axis() = -Vector3s::UnitZ();
    R1 = R0 * R.inverse();
    pAngB = R * pAngB;

    R.angle() = 0.5 * dt * pAngB.y() / I.y();
    R.axis() = -Vector3s::UnitY();
    R1 = R1 * R.inverse();
    pAngB = R * pAngB;

    R.angle() = dt * pAngB.x() / I.x(); // Note the lack of 1/2
    R.axis() = -Vector3s::UnitX();
    R1 = R1 * R.inverse();
    pAngB = R * pAngB;

    R.angle() = 0.5 * dt * pAngB.y() / I.y();
    R.axis() = -Vector3s::UnitY();
    R1 = R1 * R.inverse();
    pAngB = R * pAngB;

    R.angle() = 0.5 * dt * pAngB.z() / I.z();
    R.axis() = -Vector3s::UnitZ();
    R1 = R1 * R.inverse();
    pAngB = R * pAngB;

    // Ensure we have still have an orthonormal rotation matrix
    assert( fabs( R1.determinant() - 1.0 ) <= 1.0e-9 );
    assert( ( R1 * R1.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    // If we 're-transform' pAngB to world coordinates, we should have the momentum we started with
    #ifndef NDEBUG
    if( !( ( v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) - R1 * pAngB ).lpNorm<Eigen::Infinity>() <= 1.0e-8 ) )
    {
      std::cerr << "err: " << ( Vector3s( v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) ) - Vector3s( R1 * pAngB ) ).norm() << std::endl;
    }
    #endif
    // TODO: Check this relative to the magnitude of the momentum
    assert( ( v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) - R1 * pAngB ).lpNorm<Eigen::Infinity>() <= 1.0e-4 );
  }

  // Compute end force
  fsys.computeForce( q1, v0, next_time, F ); // Hamiltonian so there shouldn't be velocity dependent forces

  // Second momentum update
  // p1 += 0.5 * h * F_q1;
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }
    v1.segment<3>( 3 * bdy_idx ) += 0.5 * dt * F.segment<3>( 3 * bdy_idx );
    v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) += 0.5 * dt * F.segment<3>( 3 * nbodies + 3 * bdy_idx );
  }

  // Grab vector of inverse reference frame mass/inertia
  const Eigen::Map<const VectorXs,Eigen::Aligned> Minv0_vals{ fsys.Minv0().valuePtr(), fsys.Minv0().nonZeros() };

  // Convert momentum to velocity
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }
    // Linear component
    v1.segment<3>( 3 * bdy_idx ).array() /= M0_vals.segment<3>( 3 * bdy_idx ).array();
    // Rotational component
    const Eigen::Map<const Matrix33sr> R{ &q1.data()[ 3 * nbodies + 9 * bdy_idx ] };
    assert( fabs( R.determinant() - 1.0 ) < 1.0e-9 );
    assert( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() < 1.0e-9 );
    const Eigen::Map<const Vector3s> Iinv0{ &Minv0_vals.data()[ 3 * nbodies + 3 * bdy_idx ] };
    assert( ( Iinv0.array() > 0.0 ).all() );
    v1.segment<3>( 3 * nbodies + 3 * bdy_idx ) = R * Iinv0.asDiagonal() * R.transpose() * v1.segment<3>( 3 * nbodies + 3 * bdy_idx );
  }
}

void SplitHamMap::linearInertialConfigurationUpdate( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 )
{
  IntegrationUtils::exponentialEuler( q0, v0, dt, q1 );
}

std::string SplitHamMap::name() const
{
  return "split_ham";
}

void SplitHamMap::serialize( std::ostream& output_stream ) const
{
  // No-op
}
