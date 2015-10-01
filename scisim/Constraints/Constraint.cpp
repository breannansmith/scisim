// Constraint.cpp
//
// Breannan Smith
// Last updated: 09/30/2015

#include "Constraint.h"

#include <iostream>

void Constraint::computeBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  computeContactBasis( q, v, basis );
  assert( basis.rows() == basis.cols() );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( basis.rows(), basis.cols() ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );
}

void Constraint::computeForcingTerm( const VectorXs& q, const VectorXs& v, const MatrixXXsc& basis, const scalar& CoR, const scalar& nrel, const VectorXs& drel, VectorXs& constant_term ) const
{
  assert( basis.rows() == basis.cols() );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( basis.rows(), basis.cols() ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );
  assert( CoR >= 0.0 ); assert( CoR <= 1.0 );

  constant_term = VectorXs::Zero( basis.rows() );
  assert( fabs( evalNdotV( q, v ) - basis.col(0).dot( computeRelativeVelocity( q, v ) ) ) <= 1.0e-6 );
  //constant_term += basis.col(0) * ( CoR * ( evalNdotV( q, v ) - nrel ) + ( 1.0 + CoR ) * nrel );
  constant_term += basis.col(0) * ( CoR * evalNdotV( q, v ) + nrel );
  assert( drel.size() == basis.cols() - 1 );
  for( unsigned friction_sample = 0; friction_sample < drel.size(); ++friction_sample )
  {
    constant_term += basis.col( friction_sample + 1 ) * drel( friction_sample );
  }
}

scalar Constraint::penetrationDepth( const VectorXs& q ) const
{
  const scalar pen_depth = computePenetrationDepth( q );
  // nan indicates computePenetrationDepth is not implemented for a particular constraint variant
  assert( pen_depth <= 0.0 || std::isnan( pen_depth ) );
  return pen_depth;
}

scalar Constraint::overlapVolume( const VectorXs& q ) const
{
  const scalar overlap_volume = computeOverlapVolume( q );
  // nan indicates computeOverlapVolume is not implemented for a particular constraint variant
  assert( overlap_volume >= 0.0 || std::isnan( overlap_volume ) );
  return overlap_volume;
}

int Constraint::simulatedBody0() const
{
  std::pair<int,int> bodies;
  getSimulatedBodyIndices( bodies );
  return bodies.first;
}

int Constraint::simulatedBody1() const
{
  std::pair<int,int> bodies;
  getSimulatedBodyIndices( bodies );
  return bodies.second;
}

void Constraint::setSimulatedBody0( const unsigned idx )
{
  assert( simulatedBody0() >= 0 );
  setBodyIndex0( idx );
}

void Constraint::setSimulatedBody1( const unsigned idx )
{
  assert( simulatedBody1() >= 0 );
  setBodyIndex1( idx );
}

void Constraint::evalKinematicRelVelGivenBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& bases, VectorXs& nrel, VectorXs& drel )
{
  assert( bases.cols() == nrel.size() + drel.size() );
  assert( nrel.size() % constraints.size() == 0 );
  assert( drel.size() % constraints.size() == 0 );

  // Number of constraints in the system
  const unsigned ncons{ static_cast<unsigned>( constraints.size() ) };

  // Number of tangent friction samples per constraint in the system
  const unsigned friction_vectors_per_con{ static_cast<unsigned>( drel.size() / ncons ) };

  for( unsigned con_num = 0; con_num < ncons; ++con_num )
  {
    // Grab the kinematic relative velocity
    const VectorXs kinematic_rel_vel{ constraints[con_num]->computeKinematicRelativeVelocity( q, v ) };
    assert( kinematic_rel_vel.size() == bases.rows() );

    // Compute the column of the normal in the bases matrix
    const unsigned n_idx{ ( friction_vectors_per_con + 1 ) * con_num };
    assert( n_idx < bases.cols() ); assert( fabs( bases.col( n_idx ).norm() - 1.0 ) <= 1.0e-6 );
    // Project the relative velocity onto the normal
    nrel( con_num ) = - kinematic_rel_vel.dot( bases.col( n_idx ) );
    // For each tangent friction sample
    for( unsigned friction_sample = 0; friction_sample < friction_vectors_per_con; ++friction_sample )
    {
      // Compute the column of the current friction sample in the bases matrix
      const unsigned f_idx{ ( friction_vectors_per_con + 1 ) * con_num + friction_sample + 1 };
      assert( f_idx < bases.cols() );
      assert( fabs( bases.col( f_idx ).norm() - 1.0 ) <= 1.0e-6 );
      assert( fabs( bases.col( n_idx ).dot( bases.col( f_idx ) ) ) <= 1.0e-6 );
      drel( friction_vectors_per_con * con_num + friction_sample ) = - kinematic_rel_vel.dot( bases.col( f_idx ) );
    }
  }
}

Constraint::~Constraint()
{}

void Constraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const
{
  std::cerr << "Constraint::resolveImpact not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  std::cerr << "Constraint::computeGeneralizedFrictionDisk not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  std::cerr << "Constraint::computeGeneralizedFrictionGivenTangentSample not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

int Constraint::frictionStencilSize() const
{
  std::cerr << "Constraint::frictionStencilSize not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  std::cerr << "Constraint::getSimulatedBodyIndices not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  std::cerr << "Constraint::getBodyIndices not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN )  const
{
  std::cerr << "Constraint::evalKinematicNormalRelVel not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  std::cerr << "Constraint::evalH not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  std::cerr << "Constraint::computeContactBasis not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  std::cerr << "Constraint::getWorldSpaceContactPoint not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  std::cerr << "Constraint::getWorldSpaceContactNormal not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

unsigned Constraint::getStaticObjectIndex() const
{
  std::cerr << "Constraint::getStaticObjectIndex not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

VectorXs Constraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  std::cerr << "Constraint::computeRelativeVelocity not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::setBodyIndex0( const unsigned idx )
{
  std::cerr << "Constraint::setBodyIndex0 not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void Constraint::setBodyIndex1( const unsigned idx )
{
  std::cerr << "Constraint::setBodyIndex1 not implemented for: " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}

scalar Constraint::computePenetrationDepth( const VectorXs& q ) const
{
  return SCALAR_NAN;
}

scalar Constraint::computeOverlapVolume( const VectorXs& q ) const
{
  return SCALAR_NAN;
}
