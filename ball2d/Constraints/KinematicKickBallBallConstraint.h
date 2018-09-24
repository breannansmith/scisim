#ifndef KINEMATIC_KICK_BALL_BALL_CONSTRAINT_H
#define KINEMATIC_KICK_BALL_BALL_CONSTRAINT_H

#include "BallBallConstraint.h"

class KinematicKickBallBallConstraint final : public BallBallConstraint
{

public:

  KinematicKickBallBallConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& kinematic_kick, const bool teleported );

  virtual ~KinematicKickBallBallConstraint() override;

  // Inherited from Constraint
  virtual scalar evalNdotV( const VectorXs& q, const VectorXs& v ) const override;
  virtual void evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const override;
  virtual std::string name() const override;

private:

  const Vector2s m_kinematic_kick;

  virtual VectorXs computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

  virtual scalar computePenetrationDepth( const VectorXs& q ) const override;

  virtual VectorXs computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const override;

};

#endif
