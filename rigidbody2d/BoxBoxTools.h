// BoxBoxTools.h
//
// Breannan Smith
// Last updated: 01/07/2016

// A port of the box-box contact sampling routine from ODE to 2D,
// with modifications to avoid biasing the response towards one body.

#ifndef BOX_BOX_TOOLS_H
#define BOX_BOX_TOOLS_H

#include "scisim/Math/MathDefines.h"

namespace BoxBoxTools
{

  void isActive( const Vector2s& x0, const scalar& theta0, const Vector2s& r0, const Vector2s& x1, const scalar& theta1, const Vector2s& r1, Vector2s& n, std::vector<Vector2s>& points );

}

#endif
