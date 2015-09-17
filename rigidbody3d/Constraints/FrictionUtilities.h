// FrictionUtilities.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef FRICTION_UTILITIES_H
#define FRICTION_UTILITIES_H

#include "scisim/Math/MathDefines.h"

namespace FrictionUtilities
{

  // Given a non-zero vector n returns a vector orthogonal to n
  Vector3s orthogonalVector( const Vector3s& n );

  // Given a vector n and a suggested tangent, generates multiple, equally spaced vectors orthogonal to n. If possible, the
  // first vector is aligned with suggested tangent. The number of vectors is read from the container size of vectors.
  void generateOrthogonalVectors( const Vector3s& n, std::vector<Vector3s>& vectors, const Vector3s& suggested_tangent );

}

#endif
