// KinematicObjectCircleConstraint.h
//
// Breannan Smith
// Last updated: 12/07/2015

#ifndef KINEMATIC_OBJECT_CIRCLE_CONSTRAINT_H
#define KINEMATIC_OBJECT_CIRCLE_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class KinematicObjectCircleConstraint final : public Constraint
{

public:

  KinematicObjectCircleConstraint( const unsigned sim_bdy_idx, const scalar& sim_bdy_r, const Vector2s& n, const unsigned knmtc_bdy_idx );
  virtual ~KinematicObjectCircleConstraint() override = default;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual int impactStencilSize() const override;
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

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Index of the simulated circle
  unsigned m_sim_idx;

  // Circle's radius
  const scalar m_r;

  // Normal to prevent penetration along
  const Vector2s m_n;

  // Index of the kinematic body
  const unsigned m_kinematic_index;

};

#endif
