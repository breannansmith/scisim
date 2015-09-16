// StaticCylinderBodyConstraint.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef STATIC_CYLINDER_BODY_CONSTRAINT_H
#define STATIC_CYLINDER_BODY_CONSTRAINT_H

#include "SCISim/Constraints/Constraint.h"

class StaticCylinder;

class StaticCylinderBodyConstraint final : public Constraint
{

public:

  StaticCylinderBodyConstraint( const unsigned body_index, const Vector3s& collision_point, const StaticCylinder& cyl, const unsigned cylinder_index, const VectorXs& q );
  virtual ~StaticCylinderBodyConstraint() final override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const final override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const final override;
//  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& gdotD );
//  virtual void computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D, VectorXs& gdotD );
  virtual int impactStencilSize() const final override;
  virtual int frictionStencilSize() const final override;
//  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const;
//  virtual void computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const;
//  virtual void getBodyIndices( std::pair<int,int>& bodies ) const;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const final override;
  virtual bool conservesTranslationalMomentum() const final override;
  virtual bool conservesAngularMomentumUnderImpact() const final override;
  virtual bool conservesAngularMomentumUnderImpactAndFriction() const final override;
  virtual std::string name() const final override;

  // For binary force output
//  virtual void getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point );
//  virtual void getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal );
//  virtual unsigned getStaticObjectIndex() const;

//  inline unsigned cylinderIdx() const
//  {
//    return m_cylinder_index;
//  }

//  inline unsigned sphereIdx() const
//  {
//    return unsigned( m_i );
//  }

private:

  Vector3s computeN( const VectorXs& q ) const;
  Vector3s computeCylinderCollisionPointVelocity( const VectorXs& q ) const;
//  Vector3s computeRelativeVelocityAtCylinderPoint( const VectorXs& q, const VectorXs& v ) const;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  // Index of the colliding body
  const unsigned m_body_index;

  // Impact point relative to center of mass in wold coordinates
  const Vector3s m_r;

  // Cylinder involved in this collision
  const StaticCylinder& m_cyl;
  //const unsigned m_cylinder_index;

};

#endif
