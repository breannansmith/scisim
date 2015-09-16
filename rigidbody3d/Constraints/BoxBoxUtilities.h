// BoxBoxUtilities.h
//
// Adapted from ODE
// Last updated: 09/15/2015

#ifndef BOX_BOX_UTILITIES_H
#define BOX_BOX_UTILITIES_H

#include "SCISim/Math/MathDefines.h"

// TODO: Complete cleanup of this code

namespace BoxBoxUtilities
{

void isActive( const Vector3s& cm0, const Matrix33sr& R0, const Vector3s& side0, const Vector3s& cm1, const Matrix33sr& R1, const Vector3s& side1, Vector3s& n, std::vector<Vector3s>& points );

}

#endif
