// Sobogus.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "Sobogus.h"

#include "scisim/Constraints/Constraint.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Utilities.h"

#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

#include <cassert>
#include <map>

#include <iostream>

SobogusFrictionProblem::SobogusFrictionProblem( const SobogusSolverType& solver_type )
: m_solver_type( solver_type )
, m_num_bodies()
, m_num_collisions()
, m_mfp()
, m_balls_2d()
, m_rigid_body_2d()
, m_f_in()
, m_w_in()
, m_H_0_store()
, m_H_1_store()
{}

SobogusFrictionProblem::SobogusFrictionProblem( const SobogusSolverType& solver_type, const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel )
: m_solver_type( solver_type )
{
  initialize( active_set, contact_bases, masses, q0, v0, CoR, mu, nrel, drel );
}

void SobogusFrictionProblem::initialize2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel )
{
  // Compute the number of bodies in the system
  assert( masses.size() % 4 == 0 );
  m_num_bodies = masses.size() / 4;

  // Compute the number of collisions
  m_num_collisions = active_set.size();

  // Incoming momenta
  m_f_in.resize( 2 * m_num_bodies );
  assert( v0.size() == 2 * m_num_bodies );
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    // Compute and save the momentum of the current body
    assert( masses( 4 * bdy_idx + 0 ) == masses( 4 * bdy_idx + 3 ) );
    assert( masses( 4 * bdy_idx + 1 ) == 0.0 ); assert( masses( 4 * bdy_idx + 2 ) == 0.0 );
    m_f_in.segment<2>( 2 * bdy_idx ) = - masses( 4 * bdy_idx ) * v0.segment<2>( 2 * bdy_idx );
  }

  // 'Forcing' terms (kinematic collisions, restitution)
  assert( ( CoR.array() >= 0.0 ).all() ); assert( ( CoR.array() <= 1.0 ).all() );
  m_w_in.resize( 2 * m_num_collisions );
  // Flat storage for generalized contact basis
  m_H_0_store.resize( 2 * m_num_collisions, 2 );
  m_H_1_store.resize( 2 * m_num_collisions, 2 );
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Compute the contact basis
    const MatrixXXsc contact_basis = contact_bases.block<2,2>( 0, 2 * clsn_idx );
    assert( contact_basis.rows() == contact_basis.cols() );
    assert( ( contact_basis * contact_basis.transpose() - MatrixXXsc::Identity( contact_basis.rows(), contact_basis.cols() ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( contact_basis.determinant() - 1.0 ) <= 1.0e-6 );

    // Compute the 'forcing' term
    VectorXs forcing_term;
    active_set[clsn_idx]->computeForcingTerm( q0, v0, contact_basis, CoR( clsn_idx ), nrel( clsn_idx ), drel.segment<1>( clsn_idx ), forcing_term );
    assert( forcing_term.size() == 2 );
    m_w_in.segment<2>( 2 * clsn_idx ) = forcing_term;

    // Note, format for H different from E:
    //   n^T
    //   t^T
    MatrixXXsc H0{ 2, 2 };
    MatrixXXsc H1{ 2, 2 };
    active_set[clsn_idx]->evalH( q0, contact_basis, H0, H1 );
    m_H_0_store.block<2,2>( 2 * clsn_idx, 0 ) = H0;
    m_H_1_store.block<2,2>( 2 * clsn_idx, 0 ) = H1;
  }

  // Indices of the bodies involved in the collisions
  VectorXi obj_A{ m_num_collisions };
  VectorXi obj_B{ m_num_collisions };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Save the ids of the objects involved in the collision
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );
    obj_A( clsn_idx ) = object_ids.first;
    obj_B( clsn_idx ) = object_ids.second;
  }

  assert( m_num_collisions == mu.size() );
  assert( ( mu.array() >= 0.0 ).all() );
  m_balls_2d.fromPrimal( m_num_bodies, masses, m_f_in, m_num_collisions, mu, contact_bases, m_w_in, obj_A, obj_B, m_H_0_store, m_H_1_store );
}

void SobogusFrictionProblem::initializeRigidBody2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel )
{
  // Compute the number of bodies in the system
  assert( masses.size() % 9 == 0 );
  m_num_bodies = masses.size() / 9;

  // Compute the number of collisions
  m_num_collisions = active_set.size();

  // Incoming momenta
  m_f_in.resize( 3 * m_num_bodies );
  assert( v0.size() == 3 * m_num_bodies );
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    // Compute and save the momentum of the current body
    assert( masses( 9 * bdy_idx + 0 ) == masses( 9 * bdy_idx + 4 ) ); assert( masses( 9 * bdy_idx + 8 ) > 0.0 );
    assert( masses( 9 * bdy_idx + 1 ) == 0.0 ); assert( masses( 9 * bdy_idx + 2 ) == 0.0 );
    assert( masses( 9 * bdy_idx + 3 ) == 0.0 ); assert( masses( 9 * bdy_idx + 5 ) == 0.0 );
    assert( masses( 9 * bdy_idx + 6 ) == 0.0 ); assert( masses( 9 * bdy_idx + 7 ) == 0.0 );
    m_f_in.segment<2>( 3 * bdy_idx ) = - masses( 9 * bdy_idx + 0 ) * v0.segment<2>( 3 * bdy_idx );
    m_f_in( 3 * bdy_idx + 2 ) = - masses( 9 * bdy_idx + 8 ) * v0( 3 * bdy_idx + 2 );
  }

  // 'Forcing' terms (kinematic collisions, restitution)
  assert( ( CoR.array() >= 0.0 ).all() ); assert( ( CoR.array() <= 1.0 ).all() );
  m_w_in.resize( 2 * m_num_collisions );
  // Flat storage for generalized contact basis
  m_H_0_store.resize( 2 * m_num_collisions, 3 );
  m_H_1_store.resize( 2 * m_num_collisions, 3 );
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Compute the contact basis
    const MatrixXXsc contact_basis{ contact_bases.block<2,2>( 0, 2 * clsn_idx ) };
    assert( contact_basis.rows() == contact_basis.cols() );
    assert( ( contact_basis * contact_basis.transpose() - MatrixXXsc::Identity( contact_basis.rows(), contact_basis.cols() ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( contact_basis.determinant() - 1.0 ) <= 1.0e-6 );

    // Compute the 'forcing' term
    VectorXs forcing_term;
    active_set[clsn_idx]->computeForcingTerm( q0, v0, contact_basis, CoR( clsn_idx ), nrel( clsn_idx ), drel.segment<1>( clsn_idx ), forcing_term );
    assert( forcing_term.size() == 2 );
    m_w_in.segment<2>( 2 * clsn_idx ) = forcing_term;

    // Note, format for H different from E:
    //   n^T r x n
    //   t^T r x t
    // r x n, r x t are scalars
    MatrixXXsc H0{ 2, 3 };
    MatrixXXsc H1{ 2, 3 };
    active_set[clsn_idx]->evalH( q0, contact_basis, H0, H1 );
    m_H_0_store.block<2,3>( 2 * clsn_idx, 0 ) = H0;
    m_H_1_store.block<2,3>( 2 * clsn_idx, 0 ) = H1;
  }

  // Indices of the bodies involved in the collisions
  VectorXi obj_A{ m_num_collisions };
  VectorXi obj_B{ m_num_collisions };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Save the ids of the objects involved in the collision
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );
    obj_A( clsn_idx ) = object_ids.first;
    obj_B( clsn_idx ) = object_ids.second;
  }

  assert( m_num_collisions == mu.size() );
  assert( ( mu.array() >= 0.0 ).all() );
  m_rigid_body_2d.fromPrimal( m_num_bodies, masses, m_f_in, m_num_collisions, mu, contact_bases, m_w_in, obj_A, obj_B, m_H_0_store, m_H_1_store );
}

void SobogusFrictionProblem::initialize3D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel )
{
  // Compute the number of bodies in the system
  assert( masses.size() % 36 == 0 );
  m_num_bodies = masses.size() / 36;

  // Compute the number of collisions
  m_num_collisions = active_set.size();

  // Constraint free momenta
  m_f_in.resize( 6 * m_num_bodies );
  assert( v0.size() == 6 * m_num_bodies );
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    // Compute and save the momentum of the current body
    VectorXs v{ 6 };
    v.segment<3>( 0 ) = v0.segment<3>( 3 * bdy_idx );
    v.segment<3>( 3 ) = v0.segment<3>( 3 * m_num_bodies + 3 * bdy_idx );
    m_f_in.segment<6>( 6 * bdy_idx ) = - Eigen::Map<const Matrix66sc>( &masses( 36 * bdy_idx ) ) * v ;
  }

  // 'Forcing' terms (kinematic collisions, restitution)
  m_w_in.resize( 3 * m_num_collisions );
  // Flat storage for contact basis and contact basis crossed with arms for torque
  m_H_0_store.resize( 3 * m_num_collisions, 6 );
  m_H_1_store.resize( 3 * m_num_collisions, 6 );
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Compute the contact basis
    const MatrixXXsc contact_basis{ contact_bases.block<3,3>( 0, 3 * clsn_idx ) };
    assert( contact_basis.rows() == contact_basis.cols() );
    assert( ( contact_basis * contact_basis.transpose() - MatrixXXsc::Identity( contact_basis.rows(), contact_basis.cols() ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( contact_basis.determinant() - 1.0 ) <= 1.0e-6 );

    // Compute the 'forcing' term
    VectorXs forcing_term;
    active_set[clsn_idx]->computeForcingTerm( q0, v0, contact_basis, CoR( clsn_idx ), nrel( clsn_idx ), drel.segment<2>( 2 * clsn_idx ), forcing_term );
    assert( forcing_term.size() == 3 );
    m_w_in.segment<3>( 3 * clsn_idx ) = forcing_term;

    // Format for H:
    //   n^T  \tilde{n}^T
    //   s^T  \tilde{s}^T
    //   t^T  \tilde{t}^T
    MatrixXXsc H0{ 3, 6 };
    MatrixXXsc H1{ 3, 6 };
    active_set[clsn_idx]->evalH( q0, contact_basis, H0, H1 );
    m_H_0_store.block<3,6>( 3 * clsn_idx, 0  ) = H0;
    m_H_1_store.block<3,6>( 3 * clsn_idx, 0  ) = H1;
  }

  VectorXi obj_A{ m_num_collisions };
  VectorXi obj_B{ m_num_collisions };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    // Save the ids of the objects involved in the collision
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );
    obj_A( clsn_idx ) = object_ids.first;
    obj_B( clsn_idx ) = object_ids.second;
  }

  assert( m_num_collisions == mu.size() );
  m_mfp.fromPrimal( m_num_bodies, masses, m_f_in, m_num_collisions, mu, contact_bases, m_w_in, obj_A, obj_B, m_H_0_store, m_H_1_store );
}

// TODO: Factor out code by passing in the number of dofs per body?
void SobogusFrictionProblem::initialize( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel )
{
  assert( typeid(scalar) == typeid(double) );
  assert( active_set.size() == unsigned( contact_bases.cols() / contact_bases.rows() ) );
  assert( masses.size() % 36 == 0 || masses.size() % 9 == 0 || masses.size() % 4 == 0 );

  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    initialize3D( active_set, contact_bases, masses, q0, v0, CoR, mu, nrel, drel );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    initialize2D( active_set, contact_bases, masses, q0, v0, CoR, mu, nrel, drel );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    initializeRigidBody2D( active_set, contact_bases, masses, q0, v0, CoR, mu, nrel, drel );
  }
}

void SobogusFrictionProblem::solve2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const VectorXs& mu, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations )
{
  assert( typeid(scalar) == typeid(double) );
  assert( tol >= 0.0 );

  assert( alpha.size() == m_num_collisions ); assert( beta.size() == m_num_collisions );
  // Initialize impulses for warm starting
  VectorXs r{ 2 * m_num_collisions };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    #ifndef NDEBUG
    // Collision basis should be ortho-normal and orientation preserving
    {
      const Matrix22sr basis{ m_H_0_store.block<2,2>( 2 * clsn_idx, 0 ) };
      assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-9 );
      assert( ( basis * basis.transpose() - Matrix22sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    }
    // For 2D balls, the basis should be the same for each body in a collision
    {
      std::pair<int,int> bodies;
      active_set[clsn_idx]->getSimulatedBodyIndices( bodies );
      if( bodies.second >= 0 )
      {
        assert( ( m_H_0_store.block<2,2>( 2 * clsn_idx, 0 ).array() == m_H_1_store.block<2,2>( 2 * clsn_idx, 0 ).array() ).all() );
      }
    }
    #endif
    const Vector2s n{ m_H_0_store.row( 2 * clsn_idx ) };
    const Vector2s t{ m_H_0_store.row( 2 * clsn_idx + 1 ) };
    r.segment<2>( 2 * clsn_idx ) = alpha(clsn_idx) * n + beta(clsn_idx) * t;
  }

  assert( vout.size() == 2 * m_num_bodies );
  error = m_balls_2d.solve( r, vout, num_iterations, 0, tol, max_iters, eval_every, true );
  succeeded = error < tol;

  // Extract the impulses
  assert( f.size() == 2 * m_num_bodies );
  f.setZero();
  assert( m_H_0_store.rows() == 2 * m_num_collisions ); assert( m_H_0_store.cols() == 2 );
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector2s n{ m_H_0_store.block<1,2>( 2 * clsn_idx + 0, 0 ) };
    const Vector2s t{ m_H_0_store.block<1,2>( 2 * clsn_idx + 1, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, t, 1.0e-6 ) );

    const scalar alpha_local{ r.segment<2>( 2 * clsn_idx ).dot( n ) };
    alpha( clsn_idx ) = alpha_local;
    if( mu(clsn_idx) == 0.0 )
    {
      #ifndef NDEBUG
      const scalar beta_local{ r.segment<2>( 2 * clsn_idx ).dot( t ) };
      assert( fabs(beta_local) <= 8.0e-15 );
      #endif
      beta( clsn_idx ) = 0.0;
    }
    else
    {
      const scalar beta_local{ r.segment<2>( 2 * clsn_idx ).dot( t ) };
      beta( clsn_idx ) = beta_local;
    }

    // Add contribution to f from current friction force to translational momentum
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );

    const Vector2s f0{ beta( clsn_idx ) * t };
    f.segment<2>( 2 * object_ids.first ) += f0;
    if( object_ids.second >= 0 )
    {
      f.segment<2>( 2 * object_ids.second ) -= f0;
    }
  }

  // TODO: Why is this code here?
  // Reorder the degrees of freedom
  VectorXs vtmp{ vout.size() };
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    vtmp.segment<2>( 2 * bdy_idx ) = vout.segment<2>( 2 * bdy_idx );
  }
  vtmp.swap( vout );
  assert( ( vtmp.array() == vout.array() ).all() );

  // Verify that we compute the error correctly
  #ifndef NDEBUG
  VectorXs f_contact{ VectorXs::Zero( 2 * m_num_collisions ) };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector2s n{ m_H_0_store.block<1,2>( 2 * clsn_idx + 0, 0 ) };
    const Vector2s t{ m_H_0_store.block<1,2>( 2 * clsn_idx + 1, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, t, 1.0e-6 ) );
    f_contact.segment<2>( 2 * clsn_idx ) += alpha( clsn_idx ) * n;
    f_contact.segment<2>( 2 * clsn_idx ) += beta( clsn_idx ) * t;
  }
  const scalar test_error{ computeError( f_contact ) };
  assert( fabs( error - test_error ) <= 1.0e-12 );
  #endif
}

void SobogusFrictionProblem::solveRigidBody2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const VectorXs& mu, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations )
{
  assert( typeid(scalar) == typeid(double) );
  assert( tol >= 0.0 );

  assert( alpha.size() == m_num_collisions ); assert( beta.size() == m_num_collisions );
  // Initialize impulses for warm starting
  VectorXs r{ 2 * m_num_collisions };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    #ifndef NDEBUG
    // Collision basis should be ortho-normal and orientation preserving
    {
      const Matrix22sr basis{ m_H_0_store.block<2,2>( 2 * clsn_idx, 0 ) };
      assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-9 );
      assert( ( basis * basis.transpose() - Matrix22sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    }
    // The basis should be the same for each body in a collision
    {
      std::pair<int,int> bodies;
      active_set[clsn_idx]->getSimulatedBodyIndices( bodies );
      if( bodies.second >= 0 )
      {
        assert( ( m_H_0_store.block<2,2>( 2 * clsn_idx, 0 ).array() == m_H_1_store.block<2,2>( 2 * clsn_idx, 0 ).array() ).all() );
      }
    }
    #endif
    const Vector2s n{ m_H_0_store.block<1,2>( 2 * clsn_idx + 0, 0 ) };
    const Vector2s t{ m_H_0_store.block<1,2>( 2 * clsn_idx + 1, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, t, 1.0e-6 ) );
    r.segment<2>( 2 * clsn_idx ) = alpha(clsn_idx) * n + beta(clsn_idx) * t;
  }

  error = m_rigid_body_2d.solve( r, vout, num_iterations, 0, tol, max_iters, eval_every, true );
  succeeded = error < tol;

  // Extract the impulses
  assert( f.size() == 3 * m_num_bodies );
  f.setZero();
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector2s n{ m_H_0_store.block<1,2>( 2 * clsn_idx + 0, 0 ) };
    const Vector2s t{ m_H_0_store.block<1,2>( 2 * clsn_idx + 1, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, t, 1.0e-6 ) );

    const scalar alpha_local{ r.segment<2>( 2 * clsn_idx ).dot( n ) };
    alpha( clsn_idx ) = alpha_local;
    if( mu(clsn_idx) == 0.0 )
    {
      #ifndef NDEBUG
      const scalar beta_local{ r.segment<2>( 2 * clsn_idx ).dot( t ) };
      assert( fabs(beta_local) <= 8.0e-15 );
      #endif
      beta( clsn_idx ) = 0.0;
    }
    else
    {
      const scalar beta_local{ r.segment<2>( 2 * clsn_idx ).dot( t ) };
      beta( clsn_idx ) = beta_local;
    }

    // Add contribution to f from current friction force to translational momentum
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );

    const Vector2s f0{ beta( clsn_idx ) * t };
    f.segment<2>( 3 * object_ids.first ) += f0;
    if( object_ids.second >= 0 )
    {
      f.segment<2>( 3 * object_ids.second ) -= f0;
    }
    // Add contribution to f from current friction force to angular momentum
    const scalar ttilde0{ m_H_0_store( 2 * clsn_idx + 1, 2 ) };
    const scalar f0_torque{ beta( clsn_idx ) * ttilde0 };
    f( 3 * object_ids.first + 2 ) += f0_torque;
    if( object_ids.second >= 0 )
    {
      const scalar ttilde1{ m_H_1_store( 2 * clsn_idx + 1, 2 ) };
      const scalar f1_torque{ beta( clsn_idx ) * ttilde1 };
      f( 3 * object_ids.second + 2 ) -= f1_torque;
    }
  }

  // TODO: Why is this code here?
  // Reorder the degrees of freedom
  VectorXs vtmp{ vout.size() };
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    vtmp.segment<3>( 3 * bdy_idx ) = vout.segment<3>( 3 * bdy_idx );
  }
  vtmp.swap( vout );
  assert( ( vtmp.array() == vout.array() ).all() );

  // Verify that we are computing the error correctly
  #ifndef NDEBUG
  VectorXs f_contact{ VectorXs::Zero( 2 * m_num_collisions ) };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector2s n{ m_H_0_store.block<1,2>( 2 * clsn_idx + 0, 0 ) };
    const Vector2s t{ m_H_0_store.block<1,2>( 2 * clsn_idx + 1, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, t, 1.0e-6 ) );
    f_contact.segment<2>( 2 * clsn_idx ) += alpha( clsn_idx ) * n;
    f_contact.segment<2>( 2 * clsn_idx ) += beta( clsn_idx ) * t;
  }
  const scalar test_error{ computeError( f_contact ) };
  assert( fabs( error - test_error ) <= 1.0e-12 );
  #endif
}

void SobogusFrictionProblem::solve3D( const std::vector<std::unique_ptr<Constraint>>& active_set, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations )
{
  assert( typeid(scalar) == typeid(double) );
  assert( tol >= 0.0 );

  assert( alpha.size() == m_num_collisions ); assert( beta.size() == 2 * m_num_collisions );
  // TODO: Warm starting goes here. Scale the basis by alpha, beta
  assert( ( alpha.array() == 0.0 ).all() );
  assert( ( beta.array() == 0.0 ).all() );
  VectorXs r{ VectorXs::Zero( 3 * m_num_collisions ) };
  error = m_mfp.solve( r, vout, num_iterations, 0, tol, max_iters, eval_every, true );
  succeeded = error < tol;

  // Extract the impulses
  assert( f.size() == 6 * m_num_bodies );
  f.setZero();
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector3s n{ m_H_0_store.block<1,3>( 3 * clsn_idx + 0, 0 ) };
    const Vector3s s{ m_H_0_store.block<1,3>( 3 * clsn_idx + 1, 0 ) };
    const Vector3s t{ m_H_0_store.block<1,3>( 3 * clsn_idx + 2, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );

    const scalar alpha_local{ r.segment<3>( 3 * clsn_idx ).dot( n ) };
    alpha( clsn_idx ) = alpha_local;
    const scalar beta0_local{ r.segment<3>( 3 * clsn_idx ).dot( s ) };
    const scalar beta1_local{ r.segment<3>( 3 * clsn_idx ).dot( t ) };
    beta( 2 * clsn_idx + 0 ) = beta0_local;
    beta( 2 * clsn_idx + 1 ) = beta1_local;

    // Add contribution to f from current friction force to translational momentum
    std::pair<int,int> object_ids;
    active_set[clsn_idx]->getSimulatedBodyIndices( object_ids );
    assert( object_ids.first != object_ids.second );
    assert( object_ids.first >= 0 );
    assert( object_ids.second >= -1 );

    const Vector3s f0{ beta0_local * s + beta1_local * t };
    f.segment<3>( 3 * object_ids.first ) += f0;
    if( object_ids.second >= 0 )
    {
      f.segment<3>( 3 * object_ids.second ) -= f0;
    }
    // Add contribution to f from current friction force to angular momentum
    const Vector3s stilde0{ m_H_0_store.block<1,3>( 3 * clsn_idx + 1, 3 ) };
    const Vector3s ttilde0{ m_H_0_store.block<1,3>( 3 * clsn_idx + 2, 3 ) };
    const Vector3s f0_torque{ beta0_local * stilde0 + beta1_local * ttilde0 };
    f.segment<3>( 3 * m_num_bodies + 3 * object_ids.first ) += f0_torque;
    if( object_ids.second >= 0 )
    {
      const Vector3s stilde1{ m_H_1_store.block<1,3>( 3 * clsn_idx + 1, 3 ) };
      const Vector3s ttilde1{ m_H_1_store.block<1,3>( 3 * clsn_idx + 2, 3 ) };
      const Vector3s f1_torque{ beta0_local * stilde1 + beta1_local * ttilde1 };
      f.segment<3>( 3 * m_num_bodies + 3 * object_ids.second ) -= f1_torque;
    }
  }

  // Reorder the degrees of freedom
  VectorXs vtmp{ vout.size() };
  for( unsigned bdy_idx = 0; bdy_idx < m_num_bodies; ++bdy_idx )
  {
    vtmp.segment<3>( 3 * bdy_idx ) = vout.segment<3>( 6 * bdy_idx );
    vtmp.segment<3>( 3 * m_num_bodies + 3 * bdy_idx ) = vout.segment<3>( 6 * bdy_idx + 3 );
  }
  vtmp.swap( vout );

  // Verify that we are computing the error correctly
  #ifndef NDEBUG
  VectorXs f_contact{ VectorXs::Zero( 3 * m_num_collisions ) };
  for( unsigned clsn_idx = 0; clsn_idx < m_num_collisions; ++clsn_idx )
  {
    const Vector3s n{ m_H_0_store.block<1,3>( 3 * clsn_idx + 0, 0 ) };
    const Vector3s s{ m_H_0_store.block<1,3>( 3 * clsn_idx + 1, 0 ) };
    const Vector3s t{ m_H_0_store.block<1,3>( 3 * clsn_idx + 2, 0 ) };
    assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );
    f_contact.segment<3>( 3 * clsn_idx ) += alpha( clsn_idx ) * n;
    f_contact.segment<3>( 3 * clsn_idx ) += beta( 2 * clsn_idx + 0 ) * s;
    f_contact.segment<3>( 3 * clsn_idx ) += beta( 2 * clsn_idx + 1 ) * t;
  }
  const scalar test_error{ computeError( f_contact ) };
  assert( fabs( error - test_error ) <= 1.0e-11 );
  #endif
}

void SobogusFrictionProblem::solve( const std::vector<std::unique_ptr<Constraint>>& active_set, const VectorXs& mu, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations )
{
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    solve3D( active_set, max_iters, eval_every, tol, alpha, beta, f, vout, succeeded, error, num_iterations );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    solve2D( active_set, mu, max_iters, eval_every, tol, alpha, beta, f, vout, succeeded, error, num_iterations );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    solveRigidBody2D( active_set, mu, max_iters, eval_every, tol, alpha, beta, f, vout, succeeded, error, num_iterations );
  }
}

scalar SobogusFrictionProblem::computeError( const VectorXs& r )
{
  switch( m_solver_type )
  {
    case SobogusSolverType::Balls2D:
    {
      assert( r.size() % 2 == 0 );
      return m_balls_2d.evalInfNormError( r );
    }
    case SobogusSolverType::RigidBody2D:
    {
      assert( r.size() % 2 == 0 );
      return m_rigid_body_2d.evalInfNormError( r );
    }
    case SobogusSolverType::RigidBodies3D:
    {
      assert( r.size() % 3 == 0 );
      return m_mfp.evalInfNormError( r );
    }
    // GCC and Intel don't realize that we've exhausted all cases and complain about no return here.
#ifndef CMAKE_DETECTED_CLANG_COMPILER
    default:
    {
      std::cerr << "Impossible code path, this is a bug. Exiting." << std::endl;
      std::exit( EXIT_FAILURE );
    }
#endif
  }
}

Sobogus::Sobogus( const SobogusSolverType& solver_type, const unsigned eval_every )
: m_solver_type( solver_type )
, m_eval_every( eval_every )
{}

Sobogus::Sobogus( std::istream& input_stream )
: m_solver_type( Utilities::deserialize<SobogusSolverType>( input_stream ) )
, m_eval_every( Utilities::deserialize<unsigned>( input_stream ) )
{}

Sobogus::~Sobogus()
{}

// Builds a vector that, given local index i in [0,nlocalbodies), gives the global index ltg[i] [0,nglobalbodies)
void buildLocalToGlobalMap( const unsigned nglobalbodies, const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXu& ltg )
{
  // Build a vector that, for each entries, states whether a body is present in this set of collisions
  std::vector<bool> body_present( nglobalbodies, false );
  for( const std::unique_ptr<Constraint>& con : active_set )
  {
    std::pair<int,int> bodies;
    con->getSimulatedBodyIndices( bodies );
    assert( bodies.first != bodies.second );
    assert( bodies.first >= 0 );
    assert( bodies.second >= -1 );
    body_present[ bodies.first ] = true;
    if( bodies.second >= 0 )
    {
      assert( bodies.second < int( nglobalbodies ) );
      body_present[ bodies.second ] = true;
    }
  }
  // Compute the number of bodies in this set of collisions
  const unsigned num_present{ static_cast<unsigned>( std::count( std::begin( body_present ), std::end( body_present ), true ) ) };
  // Build a vector whose ith entry is the global index of the ith local body
  ltg.resize( num_present );
  unsigned local_idx = 0;
  for( unsigned global_idx = 0; global_idx < nglobalbodies; ++global_idx )
  {
    if( body_present[global_idx] )
    {
      ltg( local_idx++ ) = global_idx;
    }
  }
  assert( local_idx == num_present );
}

static void extractMass2D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const SparseMatrixsc& M, VectorXs& masses )
{
  assert( M.rows() == M.cols() );
  assert( M.rows() == M.nonZeros() );
  const Eigen::Map<const VectorXs> global_masses{ M.valuePtr(), M.nonZeros() };
  masses.resize( 4 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_number{ ltg( local_body_index ) };
    assert( global_body_number < nglobalbodies );
    assert( 4 * local_body_index + 3 < masses.size() );
    assert( global_masses( 2 * global_body_number + 0 ) == global_masses( 2 * global_body_number + 1 ) );
    masses.segment<4>( 4 * local_body_index ) << global_masses( 2 * global_body_number + 0 ), 0.0, 0.0, global_masses( 2 * global_body_number + 0 );
  }
  assert( ( masses.array() >= 0.0 ).all() );
}

static void extractMass2DRigidBody( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const SparseMatrixsc& M, VectorXs& masses )
{
  assert( M.rows() == M.cols() );
  assert( M.rows() == M.nonZeros() );
  const Eigen::Map<const VectorXs> global_masses{ M.valuePtr(), M.nonZeros() };
  masses.resize( 9 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_number{ ltg( local_body_index ) };
    assert( global_body_number < nglobalbodies );
    assert( 9 * local_body_index + 8 < masses.size() );
    assert( global_masses( 3 * global_body_number + 0 ) == global_masses( 3 * global_body_number + 1 ) );
    masses.segment<9>( 9 * local_body_index ) << global_masses( 3 * global_body_number + 0 ), 0.0, 0.0,
                                                 0.0, global_masses( 3 * global_body_number + 1 ), 0.0,
                                                 0.0, 0.0, global_masses( 3 * global_body_number + 2 );
  }
  assert( ( masses.array() >= 0.0 ).all() );
}

static void extractMass3D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const SparseMatrixsc& M, VectorXs& masses )
{
  assert( M.rows() == M.cols() );
  masses.resize( 36 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_number{ ltg( local_body_index ) };
    assert( global_body_number < nglobalbodies );

    // Map flat storage to a 6 by 6 matrix
    Eigen::Map<Matrix66sc> mass_block{ &masses( 36 * local_body_index ) };

    // The total mass is in the upper left block
    mass_block.block<3,3>( 0, 0 ) = Eigen::Map<const Vector3s>{ &M.data().value( 3 * global_body_number ) }.asDiagonal();
    assert( ( mass_block.block<3,3>( 0, 0 ).array() >= 0.0 ).all() );

    // Zero off-diagonal blocks
    mass_block.block<3,3>( 0, 3 ).setZero();
    mass_block.block<3,3>( 3, 0 ).setZero();

    // The inertia is in the lower right block
    mass_block.block<3,3>( 3, 3 ) = Eigen::Map<const Matrix33sc>{ &M.data().value( 3 * nglobalbodies + 9 * global_body_number ) };
    // TODO: Eigenvalues of inertia block should be positive

    assert( ( mass_block - mass_block.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
}

void SobogusFrictionProblem::flattenMass( const SparseMatrixsc& M, VectorXs& masses )
{
  assert( M.rows() == M.cols() );

  switch( m_solver_type )
  {
    case SobogusSolverType::RigidBodies3D:
    {
      assert( M.rows() % 6 == 0 );
      const unsigned nbodies{ static_cast<unsigned>( M.rows() ) / 6 };
      masses.resize( 36 * nbodies );
      for( unsigned body_index = 0; body_index < nbodies; ++body_index )
      {
        // Map flat storage to a 6 by 6 matrix
        Eigen::Map<Matrix66sc> mass_block{ &masses( 36 * body_index ) };

        // The total mass is in the upper left block
        mass_block.block<3,3>( 0, 0 ) = Eigen::Map<const Vector3s>{ &M.data().value( 3 * body_index ) }.asDiagonal();
        assert( ( mass_block.block<3,3>( 0, 0 ).array() >= 0.0 ).all() );

        // Zero off-diagonal blocks
        mass_block.block<3,3>( 0, 3 ).setZero();
        mass_block.block<3,3>( 3, 0 ).setZero();

        // The inertia is in the lower right block
        mass_block.block<3,3>( 3, 3 ) = Eigen::Map<const Matrix33sc>{ &M.data().value( 3 * nbodies + 9 * body_index ) };
        // TODO: Eigenvalues of inertia block should be positive
        assert( ( mass_block - mass_block.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
      }
      break;
    }
    case SobogusSolverType::Balls2D:
    {
      assert( M.rows() % 2 == 0 ); assert( M.nonZeros() == M.rows() );
      // TODO: Structure should simply be m0 0 0 m0 m1 0 0 m1 ..., so just hard code that in!
      const unsigned nbodies{ static_cast<unsigned>( M.rows() ) / 2 };
      masses.resize( 4 * nbodies );
      const Eigen::Map<const VectorXs> flattened_mass{ M.valuePtr(), M.rows() };
      for( unsigned body_index = 0; body_index < nbodies; ++body_index )
      {
        assert( flattened_mass( 2 * body_index + 0 ) == flattened_mass( 2 * body_index + 1 ) );
        Eigen::Map<Matrix22sc>{ &masses( 4 * body_index ) } << flattened_mass( 2 * body_index + 0 ), 0.0, 0.0, flattened_mass( 2 * body_index + 0 );
      }
      assert( ( masses.array() >= 0.0 ).all() );
      break;
    }
    case SobogusSolverType::RigidBody2D:
    {
      assert( M.rows() % 3 == 0 ); assert( M.nonZeros() == M.rows() );
      const unsigned nbodies{ static_cast<unsigned>( M.rows() ) / 3 };
      masses.resize( 9 * nbodies );
      const Eigen::Map<const VectorXs> flattened_mass{ M.valuePtr(), M.rows() };
      assert( ( flattened_mass.array() > 0.0 ).all() );
      for( unsigned body_index = 0; body_index < nbodies; ++body_index )
      {
        assert( flattened_mass( 3 * body_index ) == flattened_mass( 3 * body_index + 1 ) );
        masses.segment<9>( 9 * body_index ) << flattened_mass( 3 * body_index ), 0.0, 0.0, 0.0, flattened_mass( 3 * body_index + 1 ), 0.0, 0.0, 0.0, flattened_mass( 3 * body_index +2 );
      }
      break;
    }
  }
}

static void extractq2D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& q0, VectorXs& q_local )
{
  q_local.resize( 2 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    q_local.segment<2>( 2 * local_body_index ) = q0.segment<2>( 2 * global_body_index );
  }
}

static void extractq2DRigidBody( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& q0, VectorXs& q_local )
{
  q_local.resize( 3 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    q_local.segment<3>( 3 * local_body_index ) = q0.segment<3>( 3 * global_body_index );
  }
}

static void extractq3D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& q0, VectorXs& q_local )
{
  q_local.resize( 12 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    q_local.segment<3>( 3 * local_body_index ) = q0.segment<3>( 3 * global_body_index );
    q_local.segment<9>( 3 * nlocalbodies + 9 * local_body_index ) = q0.segment<9>( 3 * nglobalbodies + 9 * global_body_index );
  }
}

static void extractv2D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& v0, VectorXs& v_local )
{
  v_local.resize( 2 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    assert( 2 * local_body_index + 1 < v_local.size() ); assert( 2 * global_body_index + 1 < v0.size() );
    v_local.segment<2>( 2 * local_body_index ) = v0.segment<2>( 2 * global_body_index );
  }
}

static void extractv2DRigidBody( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& v0, VectorXs& v_local )
{
  v_local.resize( 3 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    assert( 3 * local_body_index + 2 < v_local.size() ); assert( 3 * global_body_index + 2 < v0.size() );
    v_local.segment<3>( 3 * local_body_index ) = v0.segment<3>( 3 * global_body_index );
  }
}

static void extractv3D( const unsigned nlocalbodies, const unsigned nglobalbodies, const VectorXu& ltg, const VectorXs& v0, VectorXs& v_local )
{
  v_local.resize( 6 * nlocalbodies );
  for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
  {
    const unsigned global_body_index{ ltg( local_body_index ) };
    v_local.segment<3>( 3 * local_body_index ) = v0.segment<3>( 3 * global_body_index );
    v_local.segment<3>( 3 * nlocalbodies + 3 * local_body_index ) = v0.segment<3>( 3 * nglobalbodies + 3 * global_body_index );
  }
}

void Sobogus::solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error )
{
  const unsigned nglobalbodies{ fsys.numBodies() };

  // Given local index i in [0,nlocalbodies), gives the global index ltg[i] [0,nglobalbodies)
  VectorXu ltg;
  buildLocalToGlobalMap( nglobalbodies, active_set, ltg );

  const unsigned nlocalbodies{ static_cast<unsigned>( ltg.size() ) };
  assert( unsigned( alpha.size() ) == active_set.size() ); assert( beta.size() % alpha.size() == 0 );

  // Kinematic realtive velocities
  VectorXs nrel( alpha.size() );
  VectorXs drel( beta.size() );
  Constraint::evalKinematicRelVelGivenBases( q0, v0, active_set, contact_bases, nrel, drel );

  // Remap the body indices in each constraint
  {
    // Invert ltg: given a global index returns the local index
    std::map<unsigned,unsigned> gtl;
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      gtl.insert( std::pair<unsigned,unsigned>( ltg( local_body_index ), local_body_index ) );
    }
    assert( gtl.size() == unsigned( nlocalbodies ) );

    // Re-map each body's indices to the local view
    for( const std::unique_ptr<Constraint>& con : active_set )
    {
      // Re-map the first body
      {
        const std::map<unsigned,unsigned>::const_iterator new_idx = gtl.find( con->simulatedBody0() );
        assert( new_idx != gtl.end() ); assert( new_idx->first == unsigned( con->simulatedBody0() ) );
        con->setSimulatedBody0( new_idx->second );
      }
      // Re-map the second body
      if( con->simulatedBody1() >= 0 )
      {
        const std::map<unsigned,unsigned>::const_iterator new_idx = gtl.find( con->simulatedBody1() );
        assert( new_idx != gtl.end() ); assert( new_idx->first == unsigned( con->simulatedBody1() ) );
        con->setSimulatedBody1( new_idx->second );
      }
    }
  }

  // Collect the masses for this group of bodies
  VectorXs masses;
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    extractMass3D( nlocalbodies, nglobalbodies, ltg, M, masses );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    extractMass2D( nlocalbodies, nglobalbodies, ltg, M, masses );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    extractMass2DRigidBody( nlocalbodies, nglobalbodies, ltg, M, masses );
  }

  // Collect the configuration for this group of bodies
  VectorXs q_local;
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    extractq3D( nlocalbodies, nglobalbodies, ltg, q0, q_local );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    extractq2D( nlocalbodies, nglobalbodies, ltg, q0, q_local );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    extractq2DRigidBody( nlocalbodies, nglobalbodies, ltg, q0, q_local );
  }

  // Collect the velocity for this group of bodies
  VectorXs v_local;
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    extractv3D( nlocalbodies, nglobalbodies, ltg, v0, v_local );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    extractv2D( nlocalbodies, nglobalbodies, ltg, v0, v_local );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    extractv2DRigidBody( nlocalbodies, nglobalbodies, ltg, v0, v_local );
  }

  SobogusFrictionProblem sfp{ m_solver_type, active_set, contact_bases, masses, q_local, v_local, CoR, mu, nrel, drel };

  VectorXs v_local_out;
  VectorXs f_local;
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    v_local_out.resize( 6 * nlocalbodies );
    f_local.resize( 6 * nlocalbodies );
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    v_local_out.resize( 2 * nlocalbodies );
    f_local.resize( 2 * nlocalbodies );
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    v_local_out.resize( 3 * nlocalbodies );
    f_local.resize( 3 * nlocalbodies );
  }

  {
    unsigned num_iterations;
    sfp.solve( active_set, mu, max_iters, m_eval_every, tol, alpha, beta, f_local, v_local_out, solve_succeeded, error, num_iterations );
  }

  // TODO: Convert the following to functions like above

  // Map local velocity solution to global
  vout = v0;
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      vout.segment<3>( 3 * global_body_index ) = v_local_out.segment<3>( 3 * local_body_index );
      vout.segment<3>( 3 * nglobalbodies +  3 * global_body_index ) = v_local_out.segment<3>( 3 * nlocalbodies + 3 * local_body_index );
    }
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      vout.segment<2>( 2 * global_body_index ) = v_local_out.segment<2>( 2 * local_body_index );
    }
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      vout.segment<3>( 3 * global_body_index ) = v_local_out.segment<3>( 3 * local_body_index );
    }
  }

  // Map local force solution to global
  f.setZero();
  if( m_solver_type == SobogusSolverType::RigidBodies3D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      f.segment<3>( 3 * global_body_index ) = f_local.segment<3>( 3 * local_body_index );
      f.segment<3>( 3 * nglobalbodies +  3 * global_body_index ) = f_local.segment<3>( 3 * nlocalbodies + 3 * local_body_index );
    }
  }
  else if( m_solver_type == SobogusSolverType::Balls2D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      f.segment<2>( 2 * global_body_index ) = f_local.segment<2>( 2 * local_body_index );
    }
  }
  else if( m_solver_type == SobogusSolverType::RigidBody2D )
  {
    for( unsigned local_body_index = 0; local_body_index < nlocalbodies; ++local_body_index )
    {
      const unsigned global_body_index{ ltg( local_body_index ) };
      f.segment<3>( 3 * global_body_index ) = f_local.segment<3>( 3 * local_body_index );
    }
  }

  // Reset the constraint indices
  for( std::vector<std::unique_ptr<Constraint>>::size_type con_idx = 0; con_idx < active_set.size(); ++con_idx )
  {
    // Re-map the first body
    {
      assert( active_set[con_idx]->simulatedBody0() >= 0 ); assert( active_set[con_idx]->simulatedBody0() < int( nlocalbodies ) );
      const unsigned global_body_number = ltg( active_set[con_idx]->simulatedBody0() );
      // todo assert global body number is less than num global bodies
      active_set[con_idx]->setSimulatedBody0( global_body_number );
    }
    // Re-map the second body
    if( active_set[con_idx]->simulatedBody1() >= 0 )
    {
      assert( active_set[con_idx]->simulatedBody1() < int( nlocalbodies ) );
      const unsigned global_body_number = ltg( active_set[con_idx]->simulatedBody1() );
      // todo assert global body number is less than num global bodies
      active_set[con_idx]->setSimulatedBody1( global_body_number );
    }
  }
}

unsigned Sobogus::numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const
{
  return ambient_space_dimensions - 1;
}

void Sobogus::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeBuiltInType( m_solver_type, output_stream );
  Utilities::serializeBuiltInType( m_eval_every, output_stream );
}

std::string Sobogus::name() const
{
  return "sobogus";
}
