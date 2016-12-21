#ifndef COLLISION_DETECTION_UTILITIES_H
#define COLLISION_DETECTION_UTILITIES_H

#include "scisim/Math/MathDefines.h"

namespace CollisionDetectionUtilities
{
  // Generates the quadratic to solve to determine if a ball vs. ball continuous time collision occurs.
  Vector3s computeCCDQuadraticCoeffs( const Vector2s& q0a, const Vector2s& q1a, const scalar& ra, const Vector2s& q0b, const Vector2s& q1b, const scalar& rb );

  // Given the quadratic continuous time polynomial coefficients, returns true and the first collision time if a collision occurs, or false if no collision occurs.
  std::pair<bool,scalar> ballBallCCDCollisionHappens( const Vector3s& c );
}

#endif
