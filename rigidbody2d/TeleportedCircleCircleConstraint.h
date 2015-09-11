// TeleportedCircleCircleConstraint.h
//
// Breannan Smith
// Last updated: 09/10/2015

#ifndef TELEPORTED_CIRCLE_CIRCLE_CONSTRAINT_H
#define TELEPORTED_CIRCLE_CIRCLE_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class TeleportedCircleCircleConstraint : public Constraint
{

public:

  TeleportedCircleCircleConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& delta0, const Vector2s& delta1, const scalar& radius0, const scalar& radius1 );
  virtual ~TeleportedCircleCircleConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const final override;
  virtual int impactStencilSize() const final override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const final override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const final override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const final override;
  virtual bool conservesTranslationalMomentum() const final override;
  virtual bool conservesAngularMomentumUnderImpact() const final override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const final override;
  virtual std::string name() const override;

  // For binary force output
  // TODO: We might want to change the output format to accept two points to account for portals
  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const override;

  bool operator==( const TeleportedCircleCircleConstraint& other ) const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const final override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;
  virtual void setBodyIndex1( const unsigned idx ) override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

protected:

  // Indices of the colliding bodies
  unsigned m_idx0;
  unsigned m_idx1;

  // Collision normal
  const Vector2s m_n;

  // Contact points relative to centers of mass
  const Vector2s m_r0;
  const Vector2s m_r1;

  // Displacements from the un-teleported to teleported positions for each body
  const Vector2s m_delta0;
  const Vector2s m_delta1;

  // Radii of the circles
  const scalar m_radius0;
  const scalar m_radius1;

};

#endif
