#ifndef STATIC_PLANE_BODY_CONSTRAINT_H
#define STATIC_PLANE_BODY_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class RigidBody2DStaticPlane;

class StaticPlaneBodyConstraint final : public Constraint
{

public:

  StaticPlaneBodyConstraint( const unsigned body_idx, const Vector2s& body_space_arm, const RigidBody2DStaticPlane& plane, const unsigned plane_idx );
  virtual ~StaticPlaneBodyConstraint() override = default;

  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual int impactStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual unsigned getStaticObjectIndex() const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual bool conservesTranslationalMomentum() const override;
  virtual bool conservesAngularMomentumUnderImpact() const override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const override;
  virtual std::string name() const override;
  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const override;
  virtual void getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const override;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;
  virtual void setBodyIndex0( const unsigned idx ) override;
  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  Vector2s computePlaneCollisionPointVelocity( const VectorXs& q ) const;

  // Index of the colliding body
  unsigned m_idx_body;

  // Body-space collision arm
  const Vector2s m_body_r;

  // The static plane invovled in the collision
  const RigidBody2DStaticPlane& m_plane;

  // Index of plane involved in this collision. Used for constraint cache.
  const unsigned m_idx_plane;

};

#endif
