// DMVMap.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "DMVMap.h"

#include "scisim/UnconstrainedMaps/FlowableSystem.h"

#include <iostream>

DMVMap::~DMVMap()
{}

static void solveDMV( const Vector3s& am, const scalar& h, const Vector3s& I0, Eigen::Quaternion<scalar>& q )
{
  const scalar eps{ fabs( h * 1e-15 ) };
  const scalar ha{ h / 2.0 };
  const scalar fac1{ ( I0.y() - I0.z() ) / I0.x() };
  const scalar fac2{ ( I0.z() - I0.x() ) / I0.y() };
  const scalar fac3{ ( I0.x() - I0.y() ) / I0.z() };

  const scalar am1i{ am.x() * ha / I0.x() };
  const scalar am2i{ am.y() * ha / I0.y() };
  const scalar am3i{ am.z() * ha / I0.z() };
  scalar cm1{ am1i + fac1 * am2i * am3i };
  scalar cm2{ am2i + fac2 * cm1 * am3i };
  scalar cm3{ am3i + fac3 * cm1 * cm2 };
  scalar err{ SCALAR_INFINITY };
  for( unsigned dmv_itr = 0; dmv_itr < 50; ++dmv_itr )
  {
    const scalar cm1b{ cm1 };
    const scalar cm2b{ cm2 };
    const scalar cm3b{ cm3 };
    const scalar calpha{ cm1 * cm1 + 1.0 + cm2 * cm2 + cm3 * cm3 };
    cm1 = calpha * am1i + fac1 * cm2 * cm3;
    cm2 = calpha * am2i + fac2 * cm1 * cm3;
    cm3 = calpha * am3i + fac3 * cm1 * cm2;
    err = fabs( cm1b - cm1 ) + fabs( cm2b - cm2 ) + fabs( cm3b - cm3 );
    if( err <= eps )
    {
      break;
    }
  }
  if( err > eps )
  {
    std::cerr << "Warning, DMV failed to terminate." << std::endl;
  }

  const scalar q0{ q.w() };
  const scalar q1{ q.x() };
  const scalar q2{ q.y() };
  const scalar q3{ q.z() };
  q.w() = q0 - cm1 * q1 - cm2 * q2 - cm3 * q3;
  q.x() = q1 + cm1 * q0 + cm3 * q2 - cm2 * q3;
  q.y() = q2 + cm2 * q0 + cm1 * q3 - cm3 * q1;
  q.z() = q3 + cm3 * q0 + cm2 * q1 - cm1 * q2;

  q.normalize();
}

#ifndef NDEBUG
static bool isUnitLength( const Eigen::Quaternion<scalar>& q )
{
  const scalar squared_norm = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();
  return fabs( squared_norm - 1.0 ) <= 1.0e-9;
}
#endif

static void DMV( const VectorXs& q0, const VectorXs& p, const scalar& h, const Eigen::Map<const VectorXs,Eigen::Aligned>& M0vals, const int body_num, VectorXs& q1 )
{
  assert( q0.size() % 12 == 0 ); assert( q0.size() /2 == p.size() ); assert( h > 0.0 ); assert( M0vals.size() == p.size() );
  assert( body_num >= 0 ); assert( body_num < q0.size() / 12 ); assert( q0.size() == q1.size() );

  const unsigned nbodies{ static_cast<unsigned>( q0.size() / 12 ) };

  // Get the body-frame diagonalized inertia tensor
  const Vector3s inertia{ M0vals.segment<3>( 3 * nbodies + 3 * body_num ) };
  assert( ( inertia.array() > 0.0 ).all() );

  // Get the orientation of the body
  const Eigen::Map<const Matrix33sr> R0{ &q0.data()[ 3 * nbodies + 9 * body_num ] };
  assert( fabs( R0.determinant() - 1.0 ) <= 1.0e-9 );
  assert( ( R0 * R0.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );

  // Compute the body-space angular momentum
  const Vector3s p_i{ R0.transpose() * p.segment<3>( 3 * nbodies + 3 * body_num ) };

  // Convert the input orientation to a quaternion
  Eigen::Quaternion<scalar> Q_new{ R0 };
  assert( isUnitLength( Q_new ) );
  solveDMV( p_i, h, inertia, Q_new );
  assert( isUnitLength( Q_new ) );

  // Convert the output orientation to a matrix
  Eigen::Map<Matrix33sr> R1{ &q1.data()[ 3 * nbodies + 9 * body_num ] };
  R1 = Q_new.toRotationMatrix();
  // Ensure we have still have an orthonormal rotation matrix
  assert( fabs( R1.determinant() - 1.0 ) <= 1.0e-9 );
  assert( ( R1 * R1.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
}

void DMVMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
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
    const int nbodies{ static_cast<int>( q0.size() / 12 ) };
    for( int i = 0; i < nbodies; ++i )
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

  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      continue;
    }
    DMV( q0, v1, dt, M0_vals, bdy_idx, q1 );
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

std::string DMVMap::name() const
{
  return "dmv";
}

void DMVMap::serialize( std::ostream& output_stream ) const
{
  // No-op
}
