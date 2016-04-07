// CircleBoxTools.h
//
// Breannan Smith
// Last updated: 04/06/2016

#ifndef CIRCLE_BOX_TOOLS_H
#define CIRCLE_BOX_TOOLS_H

#include "scisim/Math/MathDefines.h"

namespace CircleBoxTools
{

  bool isActive( const Vector2s& x0, const scalar& r0, const Vector2s& x1, const scalar& theta1, const Vector2s& r1, Vector2s& n, Vector2s& p );

}

#endif
