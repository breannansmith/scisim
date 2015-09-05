// BallStaticPlaneConstraint.h
//
// Breannan Smith
// Last updated: 09/04/2015

#ifndef STATIC_PLANE_CONSTRAINT_H
#define STATIC_PLANE_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class StaticPlane;

class StaticPlaneConstraint final : public Constraint
{

public:

  static bool isActive( const unsigned ball_idx, const VectorXs& q, const VectorXs& r, const Vector2s& x, const Vector2s& n );

  StaticPlaneConstraint( const unsigned ball_idx, const scalar& r, const StaticPlane& static_plane, const unsigned plane_idx );
  virtual ~StaticPlaneConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  //virtual void resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, VectorXs& vout ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& gdotD ) const override;
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
  virtual unsigned getStaticObjectIndex() const override;

  unsigned planeIdx() const;
  unsigned ballIdx() const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  virtual VectorXs projectImpulseOnFrictionBasis( const VectorXs& q, const VectorXs& f ) const override;
  virtual bool basisSpansTangent() const override;

  Vector2s computePlaneCollisionPointVelocity( const VectorXs& q ) const;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Index of the ball
  unsigned m_ball_idx;

  // Ball's radius; used in force output and penetration depth
  const scalar m_r;

  // Plane involved in this collision
  const StaticPlane& m_static_plane;

  // Index of plane involved in this collision; used for constraint cache and force output
  const unsigned m_idx_plane;

};

#endif
