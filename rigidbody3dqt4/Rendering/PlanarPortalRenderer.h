#ifndef PLANAR_PORTAL_RENDERER_H
#define PLANAR_PORTAL_RENDERER_H

#include "SCISim/Math/MathDefines.h"

class PlanarPortal;
class StaticPlane;

class PlanarPortalRenderer final
{

public:

  // idx: index of the planar portal
  // half_width0: half-width of the first plane
  // half_width1: half-width of the second plane
  // If half_width* == 0, nothing is rendered
  PlanarPortalRenderer( const unsigned idx, const Array2s& half_width0, const Array2s& half_width1 );

  unsigned idx() const;

  void draw( const PlanarPortal& planar_portal ) const;

private:

  void drawHalfPlane( const StaticPlane& plane, const Array2s& half_width ) const;

  unsigned m_idx;
  Array2s m_half_width0;
  Array2s m_half_width1;

};

#endif
