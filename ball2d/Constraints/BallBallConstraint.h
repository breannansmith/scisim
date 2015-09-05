// BallBallConstraint.h
//
// Breannan Smith
// Last updated: 09/03/2015

// TODO: Break BallBallConstraint and TeleportedBallBallConstraint into separate classes

#ifndef BALL_BALL_CONSTRAINT_H
#define BALL_BALL_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class BallBallConstraint : public Constraint
{

public:

  static bool isActive( const unsigned idx0, const unsigned idx1, const VectorXs& q, const VectorXs& r );
  static bool isActive( const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1 );

  BallBallConstraint( const unsigned idx0, const unsigned idx1, const VectorXs& q, const scalar& r0, const scalar& r1, const bool teleported );
  BallBallConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const bool teleported );

  virtual ~BallBallConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  //virtual void resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, VectorXs& vout ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
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

  // Indices of the colliding balls
  unsigned idx0() const;
  unsigned idx1() const;

  bool teleported() const;

protected:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;
  virtual void setBodyIndex1( const unsigned idx ) override;

  virtual VectorXs projectImpulseOnFrictionBasis( const VectorXs& q, const VectorXs& f ) const override;
  virtual bool basisSpansTangent() const override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Indices of the colliding balls
  unsigned m_sphere_idx0;
  unsigned m_sphere_idx1;

  // Collision normal
  const Vector2s m_n;

  // Radii of the balls; used for force output
  const scalar m_r0;
  const scalar m_r1;

  // True if collision occured through a portal
  const bool m_teleported;

};

#endif
