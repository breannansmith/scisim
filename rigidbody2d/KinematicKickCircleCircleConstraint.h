#ifndef KINEMATIC_KICK_CIRCLE_CIRCLE_CONSTRAINT_H
#define KINEMATIC_KICK_CIRCLE_CIRCLE_CONSTRAINT_H

#include "TeleportedCircleCircleConstraint.h"

class KinematicKickCircleCircleConstraint final : public TeleportedCircleCircleConstraint
{

public:

  KinematicKickCircleCircleConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& kinematic_kick );

  virtual ~KinematicKickCircleCircleConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual std::string name() const override;

private:

  const Vector2s m_kinematic_kick;

  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

};

#endif
