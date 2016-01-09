// BodyBodyConstraint.h
//
// Breannan Smith
// Last updated: 01/07/2016

#ifndef BODY_BODY_CONSTRAINT_H
#define BODY_BODY_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

class BodyBodyConstraint final : public Constraint
{

public:

  // idx0: index of the first body
  // idx1: index of the second body
  // p: contact point
  // n: contact normal
  // q: configuration
  BodyBodyConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& p, const Vector2s& n, const VectorXs& q );
  virtual ~BodyBodyConstraint() override = default;

  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual int impactStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
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
  virtual void setBodyIndex1( const unsigned idx ) override;
  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Indices of the colliding bodies
  unsigned m_idx0;
  unsigned m_idx1;

  // Collision normal (world space)
  const Vector2s m_n;

  // Collision arms for each body (world space)
  const Vector2s m_r0;
  const Vector2s m_r1;

};

#endif
