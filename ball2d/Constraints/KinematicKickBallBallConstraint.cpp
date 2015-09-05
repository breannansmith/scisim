// KinematicKickBallBallConstraint.cpp
//
// Breannan Smith
// Last updated: 09/04/2015

#include "KinematicKickBallBallConstraint.h"

KinematicKickBallBallConstraint::KinematicKickBallBallConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& kinematic_kick, const bool teleported )
: BallBallConstraint( idx0, idx1, x0, x1, r0, r1, teleported )
, m_kinematic_kick( kinematic_kick )
{}

KinematicKickBallBallConstraint::~KinematicKickBallBallConstraint()
{}

scalar KinematicKickBallBallConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( 2 * m_sphere_idx0 + 1 < v.size() ); assert( 2 * m_sphere_idx1 + 1 < v.size() );
  return m_n.dot( v.segment<2>( 2 * m_sphere_idx0 ) - v.segment<2>( 2 * m_sphere_idx1 ) - m_kinematic_kick );
}

void KinematicKickBallBallConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );
  gdotN( strt_idx ) = - m_n.dot( m_kinematic_kick );
}

std::string KinematicKickBallBallConstraint::name() const
{
  if( m_teleported )
  {
    return "teleported_kinematic_kick_ball_ball";
  }
  else
  {
    return "kinematic_kick_ball_ball";
  }
}

VectorXs KinematicKickBallBallConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( 2 * m_sphere_idx0 + 1 < v.size() ); assert( 2 * m_sphere_idx1 + 1 < v.size() );
  return v.segment<2>( 2 * m_sphere_idx0 ) - v.segment<2>( 2 * m_sphere_idx1 ) - m_kinematic_kick;
}

scalar KinematicKickBallBallConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  if( m_teleported )
  {
    return SCALAR_NAN;
  }
  else
  {
    return std::min( 0.0, ( q.segment<2>( 2 * m_sphere_idx0 ) - q.segment<2>( 2 * m_sphere_idx1 ) ).norm() - ( m_r0 + m_r1 ) );
  }
}

VectorXs KinematicKickBallBallConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  return m_kinematic_kick;
}
