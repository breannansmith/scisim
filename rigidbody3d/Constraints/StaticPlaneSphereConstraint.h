// StaticPlaneSphereConstraint.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef STATIC_PLANE_SPHERE_CONSTRAINT_H
#define STATIC_PLANE_SPHERE_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class StaticPlane;

class StaticPlaneSphereConstraint final : public Constraint
{

public:

  static bool isActive( const Vector3s& x_plane, const Vector3s& n_plane, const Vector3s& x_sphere, const scalar& r );

  StaticPlaneSphereConstraint( const unsigned sphere_idx, const scalar& r, const StaticPlane& plane, const unsigned plane_idx );
  virtual ~StaticPlaneSphereConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const override;
  virtual void computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const override;
  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const override;
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

  const StaticPlane& plane() const;
  unsigned planeIdx() const;
  unsigned sphereIdx() const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  Vector3s computePlaneCollisionPointVelocity( const VectorXs& q ) const;

  // Index of the colliding sphere
  unsigned m_sphere_idx;

  // Sphere's radius
  const scalar m_r;

  // Plane involved in this collision
  const StaticPlane& m_plane;

  // Index of plane involved in this collision. Used for constraint cache.
  const unsigned m_plane_idx;

};

#endif
