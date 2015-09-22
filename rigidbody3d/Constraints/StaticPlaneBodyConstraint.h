// StaticPlaneBodyConstraint.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef STATIC_PLANE_BODY_CONSTRAINT_H
#define STATIC_PLANE_BODY_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class StaticPlaneBodyConstraint final : public Constraint
{

public:

  StaticPlaneBodyConstraint( const unsigned body_idx, const Vector3s& collision_point, const Vector3s& n, const VectorXs& q, const unsigned plane_idx );
  virtual ~StaticPlaneBodyConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const override;
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

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Index of the colliding body
  unsigned m_idx_body;

  // Collision normal
  const Vector3s m_n;

  // Impact point relative to center of mass in wold coordinates
  const Vector3s m_r;

  // Index of plane involved in this collision. Used for constraint cache.
  const unsigned m_idx_plane;

};

#endif
