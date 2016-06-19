// StaticCylinderBodyConstraint.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef STATIC_CYLINDER_BODY_CONSTRAINT_H
#define STATIC_CYLINDER_BODY_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class StaticCylinder;

class StaticCylinderBodyConstraint final : public Constraint
{

public:

  StaticCylinderBodyConstraint( const unsigned body_index, const Vector3s& collision_point, const StaticCylinder& cyl, const unsigned cylinder_index, const VectorXs& q );
  virtual ~StaticCylinderBodyConstraint() final override = default;

  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const final override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const final override;
  virtual int impactStencilSize() const final override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual bool conservesTranslationalMomentum() const final override;
  virtual bool conservesAngularMomentumUnderImpact() const final override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const final override;
  virtual std::string name() const final override;

private:

  virtual void computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const override;
  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual void setBodyIndex0( const unsigned idx ) override;

  Vector3s computeN( const VectorXs& q ) const;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Index of the colliding body
  unsigned m_idx_body;

  // Impact point relative to center of mass in wold coordinates
  const Vector3s m_r;

  // Cylinder involved in this collision
  const StaticCylinder& m_cyl;

};

#endif
