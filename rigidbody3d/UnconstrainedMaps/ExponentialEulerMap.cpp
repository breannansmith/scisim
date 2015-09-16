// ExponentialEulerMap.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "ExponentialEulerMap.h"

#include "SCISim/UnconstrainedMaps/FlowableSystem.h"

ExponentialEulerMap::~ExponentialEulerMap()
{}

static void projectOrientation( const unsigned i, VectorXs& q )
{
  assert( q.size() % 12 == 0 );

  const unsigned nb{ static_cast<unsigned>( q.size() / 12 ) };

  Eigen::JacobiSVD<Matrix33sr> svd;

  Matrix33sr R{ Eigen::Map<Matrix33sr>{ q.segment<9>( 3 * nb + 9 * i ).data() } };
  svd.compute( R, Eigen::ComputeFullU | Eigen::ComputeFullV );
  Eigen::Map<Matrix33sr>{ q.segment<9>( 3 * nb + 9 * i ).data() } = svd.matrixU() * svd.matrixV().transpose();

  #ifndef NDEBUG
  {
    R = Eigen::Map<Matrix33sr>{ q.segment<9>( 3 * nb + 9 * i ).data() };
    assert( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() < 1.0e-6 );
    assert( fabs( R.determinant() - 1.0 ) < 1.0e-6 );
  }
  #endif
}

void ExponentialEulerMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  assert( iteration > 0 );
  const scalar next_time{ iteration * dt };

  // Until kinematic sripting support is added, make sure code not called
  #ifndef NDEBUG
  {
    const unsigned nbodies{ static_cast<unsigned>(q0.size() / 12) };
    for( unsigned i = 0; i < nbodies; ++i )
    {
      assert( !fsys.isKinematicallyScripted( i ) );
    }
  }
  #endif

  assert( q0.size() % 12 == 0 );
  assert( q0.size() == 2 * v0.size() );

  const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };

  // For each body
  for( unsigned i = 0; i < nbodies; ++i )
  {
    // Update the center of mass position
    q1.segment<3>( 3 * i ) = q0.segment<3>( 3 * i ) + dt * v0.segment<3>( 3 * i );

    // Update the orientaiton
    const Vector3s omega{ v0.segment<3>( 3 * nbodies + 3 * i ) };

    const Eigen::Map<const Matrix33sr> R0{ q0.segment<9>( 3 * nbodies + 9 * i ).data() };
    Eigen::Map<Matrix33sr> R1{ q1.segment<9>( 3 * nbodies + 9 * i ).data() };

    for( int j = 0; j < 3; ++j )
    {
      R1.col( j ) = R0.col( j ) + dt * omega.cross( R0.col( j ) );
    }
  }

  // Compute the acceleartion at ( q0, v0 )
  VectorXs A{ v0.size() };
  fsys.computeForce( q0, v0, next_time, A );
  A = fsys.Minv() * A;

  // For each body
  for( unsigned i = 0; i < nbodies; ++i )
  {
    // Update the linear velocity
    v1.segment<3>( 3 * i ) = v0.segment<3>( 3 * i ) + dt * A.segment<3>( 3 * i );

    // Update the angular velocity
    v1.segment<3>( 3 * ( nbodies + i ) ) = v0.segment<3>( 3 * ( nbodies + i ) ) + dt * A.segment<3>( 3 * ( nbodies + i ) );
  }

  // For each body
  for( unsigned i = 0; i < nbodies; ++i )
  {
    // Project the orientation back to a rotation
    projectOrientation( i, q1 );
  }
}

std::string ExponentialEulerMap::name() const
{
  return "exponential_euler";
}

void ExponentialEulerMap::serialize( std::ostream& output_stream ) const
{
  // No state to serialize
}
