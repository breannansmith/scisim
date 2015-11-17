// IntegrationTools.cpp
//
// Breannan Smith
// Last updated: 11/15/2015

#include "IntegrationTools.h"

// TODO: Could probably make this faster by explicitly using Rodrigues' rotation formula
static void updateOrientation( const unsigned bdy_num, const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 )
{
  assert( q0.size() == q1.size() );
  assert( q0.size() % 12 == 0 );
  assert( v0.size() * 2 == q0.size() );
  assert( 12 * bdy_num < q0.size() );

  const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };

  // Extract the axis and amount of rotation
  Eigen::AngleAxis<scalar> rotation;
  {
    Vector3s axis{ v0.segment<3>( 3 * nbodies + 3 * bdy_num ) };
    scalar theta{ axis.norm() };
    if( theta != 0.0 )
    {
      axis /= theta;
    }
    theta *= dt;

    rotation.axis() = axis;
    rotation.angle() = theta;
  }

  // Start orientation
  const Eigen::Map<const Matrix33sr> Q0{ q0.segment<9>( 3 * nbodies + 9 * bdy_num ).data() };

  // Orientation we will update
  Eigen::Map<Matrix33sr> Q1{ q1.segment<9>( 3 * nbodies + 9 * bdy_num ).data() };

  Q1 = rotation * Q0;
}

void IntegrationTools::exponentialEuler( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 )
{
  assert( q0.size() == q1.size() );
  assert( q0.size() % 12 == 0 );
  assert( v0.size() * 2 == q0.size() );

  const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };

  // Center of mass update
  q1.segment( 0, 3 * nbodies ) = q0.segment( 0, 3 * nbodies ) + dt * v0.segment( 0, 3 * nbodies );

  // Orientation update
  for( unsigned bdy_num = 0; bdy_num < nbodies; ++bdy_num )
  {
    updateOrientation( bdy_num, q0, v0, dt, q1 );
  }
}
