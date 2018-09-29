#ifndef CIRCLE_CIRCLE_CONSTRAINT_H
#define CIRCLE_CIRCLE_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class CircleCircleConstraint final : public Constraint
{

public:

  static bool isActive( const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1 );

  CircleCircleConstraint( const unsigned sphere_idx0, const unsigned sphere_idx1, const Vector2s& n, const Vector2s& p, const scalar& r0, const scalar& r1 );
  virtual ~CircleCircleConstraint() override = default;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  //virtual void resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  //virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const override;
  //virtual void computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const override;
  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
  //virtual void computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual bool conservesTranslationalMomentum() const override;
  virtual bool conservesAngularMomentumUnderImpact() const override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const override;
  virtual std::string name() const override;

  // For binary force output
  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const override;
  virtual void getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const override;

  bool operator==( const CircleCircleConstraint& other ) const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;
  virtual void setBodyIndex1( const unsigned idx ) override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;
  //virtual scalar computeOverlapVolume( const VectorXs& q ) const;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Indices of the colliding balls
  unsigned m_idx0;
  unsigned m_idx1;

  // Collision normal
  const Vector2s m_n;

  // World-space point of impact
  const Vector2s m_p;

  // Radii of the spheres (for penetration depth)
  const scalar m_r0;
  const scalar m_r1;

};

#endif
