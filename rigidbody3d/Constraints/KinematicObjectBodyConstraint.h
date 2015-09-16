// KinematicObjectBodyConstraint.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef KINEMATIC_OBJECT_BODY_CONSTRAINT_H
#define KINEMATIC_OBJECT_BODY_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class KinematicObjectBodyConstraint final : public Constraint
{

public:

  // bdy_idx: index of the simulated body
  // knmtc_idx: index of the kinematic object
  // p: contact point
  // n: contact normal
  // q: configuration
  KinematicObjectBodyConstraint( const unsigned bdy_idx, const unsigned knmtc_idx, const Vector3s& p, const Vector3s& n, const VectorXs& q );
  virtual ~KinematicObjectBodyConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
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

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  unsigned m_bdy_idx;
  const unsigned m_knmtc_idx;
  const Vector3s m_n;
  const Vector3s m_r;

};

#endif
