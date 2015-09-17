#ifndef STATIC_PLANE_RENDERER_H
#define STATIC_PLANE_RENDERER_H

#include "SCISim/Math/MathDefines.h"

class StaticPlane;

class StaticPlaneRenderer final
{

public:

  StaticPlaneRenderer( const unsigned idx, const Array2s& half_width );

  unsigned idx() const;

  void draw( const StaticPlane& plane ) const;

private:

  unsigned m_idx;
  Array2s m_half_width;

};

#endif
