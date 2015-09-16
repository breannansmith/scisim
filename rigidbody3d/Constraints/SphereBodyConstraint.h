// SphereBodyConstraint.h
//
// Breannan Smith
// Last updated: 09/15/2015
// N.b. this class is effectively abandoned.

#ifndef SPHERE_BODY_CONSTRAINT_H
#define SPHERE_BODY_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class SphereBodyConstraint final : public Constraint
{

public:

  // isphere: index of the sphere
  // ibody: index of the body
  // p: contact point in world space
  // n: contact normal in world space
  SphereBodyConstraint( const int isphere, const int ibody, const Vector3s& p, const Vector3s& n, const VectorXs& q );
  virtual ~SphereBodyConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual bool conservesTranslationalMomentum() const override;
  virtual bool conservesAngularMomentumUnderImpact() const override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const override;
  virtual std::string name() const override;

private:

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Indices of the colliding bodies
  const int m_idx_sphere;
  const int m_idx_body;

  // Collision normal
  const Vector3s m_n;
  const Vector3s m_r_sphere;
  const Vector3s m_r_body;

};

#endif
