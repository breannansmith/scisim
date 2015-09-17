// KinematicObjectSphereConstraint.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef KINEMATIC_OBJECT_SPHERE_CONSTRAINT_H
#define KINEMATIC_OBJECT_SPHERE_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class KinematicObjectSphereConstraint final : public Constraint
{

public:

  // sphere_idx: index of sphere
  // r: radius of sphere
  // n: collision normal
  // kinematic_index: index of the kinematic body
  // X: center of mass of kinematic object
  // V: velocity of kinematic object
  // omega: angular velocity of kinematic object
  KinematicObjectSphereConstraint( const unsigned sphere_idx, const scalar& r, const Vector3s& n, const unsigned kinematic_index, const Vector3s& X, const Vector3s& V, const Vector3s& omega );
  virtual ~KinematicObjectSphereConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
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

  unsigned sphereIdx() const;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  //Vector3s computeKinematicCollisionPointVelocity( const VectorXs& q ) const;

  // Index of the colliding sphere
  unsigned m_sphere_idx;

  // Sphere's radius
  const scalar m_r;

  // Normal to prevent penetration along
  const Vector3s m_n;

  // Index of the kinematic body
  const unsigned m_kinematic_index;

  // Center of the kinematic object
  const Vector3s m_X;

  // Translational velocity of the kinematic object
  const Vector3s m_V;

  // Rotational velocity of the kinematic object
  const Vector3s m_omega;

};

#endif
