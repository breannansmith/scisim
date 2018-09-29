#ifndef STATIC_PLANE_CIRCLE_CONSTRAINT_H
#define STATIC_PLANE_CIRCLE_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class RigidBody2DStaticPlane;

class StaticPlaneCircleConstraint final : public Constraint
{

public:

  static bool isActive( const Vector2s& x_circle, const scalar& r_circle, const RigidBody2DStaticPlane& plane );

  StaticPlaneCircleConstraint( const unsigned body_idx, const unsigned plane_idx, const scalar& r, const RigidBody2DStaticPlane& plane );
  virtual ~StaticPlaneCircleConstraint() override = default;

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
  virtual unsigned getStaticObjectIndex() const override;

  bool operator==( const StaticPlaneCircleConstraint& other ) const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  Vector2s computePlaneCollisionPointVelocity( const VectorXs& q ) const;

  // Index of the colliding circle
  unsigned m_circle_idx;

  // Circle's radius
  const scalar m_r;

  // Plane involved in this collision
  const RigidBody2DStaticPlane& m_plane;

  // Index of plane involved in this collision. Used for constraint cache.
  const unsigned m_plane_idx;

};

#endif
