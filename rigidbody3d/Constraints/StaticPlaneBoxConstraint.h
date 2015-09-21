// StaticPlaneBoxConstraint.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef STATIC_PLANE_BOX_CONSTRAINT_H
#define STATIC_PLANE_BOX_CONSTRAINT_H

#include "scisim/Constraints/Constraint.h"

// TODO: Merge this with StaticPlaneBodyConstraint
// TODO: Update to handle kinematic planes
class StaticPlaneBoxConstraint final : public Constraint
{

public:

  static bool isActive( const Vector3s& x_plane, const Vector3s& n, const Vector3s& x_box, const Matrix33sr& R, const Vector3s& half_width, std::vector<short>& active_corners );

  StaticPlaneBoxConstraint( const unsigned box_idx, const short corner_num, const Vector3s& n, const Vector3s& half_width, const VectorXs& q, const unsigned plane_idx );
  virtual ~StaticPlaneBoxConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const override;
  virtual void computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const override;
  virtual void computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const override;
  virtual void computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const override;
  virtual int impactStencilSize() const override;
  virtual int frictionStencilSize() const override;
  virtual void getSimulatedBodyIndices( std::pair<int,int>& bodies ) const override;
  virtual void computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual void evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const override;
  virtual void getBodyIndices( std::pair<int,int>& bodies ) const override;
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

  // Index of the colliding box
  unsigned m_idx_box;

  // Collision normal
  const Vector3s m_n;

  // Impact point relative to center of mass in wold coordinates
  const Vector3s m_r;

  // Index of plane involved in this collision. Used for constraint cache.
  const unsigned m_idx_plane;

};

#endif