// TeleportedSphereSphereConstraint.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef TELEPORTED_SPHERE_SPHERE_CONSTRAINT_H
#define TELEPORTED_SPHERE_SPHERE_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class TeleportedSphereSphereConstraint final : public Constraint
{

public:

  TeleportedSphereSphereConstraint( const unsigned idx0, const unsigned idx1, const Vector3s& x0, const Vector3s& x1, const scalar& r0, const scalar& r1 );
  virtual ~TeleportedSphereSphereConstraint() override;

  unsigned idx0() const;
  unsigned idx1() const;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const override;
  virtual void computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const override;
  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual bool conservesTranslationalMomentum() const override;
  virtual bool conservesAngularMomentumUnderImpact() const override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const override;
  virtual std::string name() const override;

  // For binary force output
  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const override;
  virtual void getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const override;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;
  virtual void setBodyIndex1( const unsigned idx ) override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Indices of the colliding balls
  unsigned m_idx0;
  unsigned m_idx1;

  // Collision normal
  const Vector3s m_n;

  // Contact points relative to centers of mass
  const Vector3s m_r0;
  const Vector3s m_r1;

};

#endif
