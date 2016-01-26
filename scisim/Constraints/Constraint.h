// Constraint.h
//
// Breannan Smith
// Last updated: 09/30/2015

// TODO: This class has accumulated up lots of redundant code. Pare down to the minimum and cleanup.

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <iosfwd>
#include <memory>

#include "scisim/Math/MathDefines.h"

class FlowableSystem;

class Constraint
{

public:

  // Returns the full contact basis
  void computeBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const;

  // Computes the 'forcing' term as needed by So-bogus to handle restitution and collisions with kinematically scripted boundaries
  void computeForcingTerm( const VectorXs& q, const VectorXs& v, const MatrixXXsc& basis, const scalar& CoR, const scalar& nrel, const VectorXs& drel, VectorXs& constant_term ) const;

  scalar penetrationDepth( const VectorXs& q ) const;
  scalar overlapVolume( const VectorXs& q ) const;

  int body0() const;
  int body1() const;

  int simulatedBody0() const;
  int simulatedBody1() const;
  void setSimulatedBody0( const unsigned idx );
  void setSimulatedBody1( const unsigned idx );

  static void evalKinematicRelVelGivenBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& bases, VectorXs& nrel, VectorXs& drel );

  virtual ~Constraint() = 0;

  // Evaluates the relative velocity projected onto the constraint gradient
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const = 0;

  // Performs a pairwise response and adds response into alpha; used by pairwise response methods (e.g. Gauss-Seidel, Jacobi)
  virtual void resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const;

  // Adds this constraint to a column of a sparse matrix. Used when forming: N^T M^-1 N
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const;

  // Adds this constraint's friction disk to columns of a sparse matrix.
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const;

  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const;

  // Returns the size of the impact stencil
  virtual int impactStencilSize() const = 0;

  // Returns the size of the friction stencil
  virtual int frictionStencilSize() const;

  // Indices of *simulated* bodies. The second entry will be -1 for
  // collisions with static geometry and kinematic bodies.
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const;
  // Indices of bodies whose degrees of freedom are stored in the global configuration. Will return
  // a body index in the second entry for kinematic bodies, -1 in the second entry for static geometry.
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const;

  // Compute relative velocity due to constraints
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const;

  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const;

  virtual bool conservesTranslationalMomentum() const = 0;
  virtual bool conservesAngularMomentumUnderImpact() const = 0;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const = 0;

  virtual std::string name() const = 0;

  // For binary force output
  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const;
  virtual void getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const;
  // TODO: Formulate a better solution than this
  virtual unsigned getStaticObjectIndex() const;

  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const;
  virtual void setBodyIndex0( const unsigned idx );
  virtual void setBodyIndex1( const unsigned idx );
  virtual scalar computePenetrationDepth( const VectorXs& q ) const;
  virtual scalar computeOverlapVolume( const VectorXs& q ) const;
  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const = 0;

};

#endif
