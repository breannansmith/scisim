#include "KinematicKickCircleCircleConstraint.h"

#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

// NOTE: displacements not supported for kinematic kick circle circle constraints, yet...
KinematicKickCircleCircleConstraint::KinematicKickCircleCircleConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& kinematic_kick )
: TeleportedCircleCircleConstraint( idx0, idx1, x0, x1, r0, r1, Vector2s::Constant( SCALAR_NAN ), Vector2s::Constant( SCALAR_NAN ), SCALAR_NAN, SCALAR_NAN )
, m_kinematic_kick( kinematic_kick )
{}

KinematicKickCircleCircleConstraint::~KinematicKickCircleCircleConstraint()
{}

scalar KinematicKickCircleCircleConstraint::evalNdotV( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_idx0 + 1 < v.size() ); assert( 3 * m_idx1 + 1 < v.size() );
  // n || r => n dot ( omega cross r ) == 0
  return m_n.dot( v.segment<2>( 3 * m_idx0 ) - v.segment<2>( 3 * m_idx1 ) - m_kinematic_kick );
}

void KinematicKickCircleCircleConstraint::evalKinematicNormalRelVel( const VectorXs& /*q*/, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );
  gdotN( strt_idx ) = - m_n.dot( m_kinematic_kick );
}

std::string KinematicKickCircleCircleConstraint::name() const
{
  return "kinematic_kick_circle_circle";
}

VectorXs KinematicKickCircleCircleConstraint::computeRelativeVelocity( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );

  // Rotate 90 degrees counter clockwise for computing the torque
  assert( fabs( MathUtilities::cross( m_n, m_r0 ) ) <= 1.0e-6 );
  const Vector2s t0{ -m_r0.y(), m_r0.x() };

  // Rotate 90 degrees counter clockwise for computing the torque
  assert( fabs( MathUtilities::cross( m_n, m_r1 ) ) <= 1.0e-6 );
  const Vector2s t1{ -m_r1.y(), m_r1.x() };

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 ) - kinematic_contribution
  return v.segment<2>( 3 * m_idx0 ) + v( 3 * m_idx0 + 2 ) * t0 - v.segment<2>( 3 * m_idx1 ) - v( 3 * m_idx1 + 2 ) * t1 - m_kinematic_kick;
}

VectorXs KinematicKickCircleCircleConstraint::computeKinematicRelativeVelocity( const VectorXs& /*q*/, const VectorXs& /*v*/ ) const
{
  return m_kinematic_kick;
}
