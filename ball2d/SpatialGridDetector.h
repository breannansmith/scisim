// SpatialGridDetector.h
//
// Breannan Smith
// Last updated: 09/04/2015

#ifndef SPATIAL_GRID_DETECTOR_H
#define SPATIAL_GRID_DETECTOR_H

#include "SCISim/Math/MathDefines.h"

#include <set>
#include <vector>

class AABB final
{

public:

  AABB();
  AABB( const Array2s& min, const Array2s& max );

  bool overlaps( const AABB& other ) const;

  inline Array2s& min() { return m_min; }
  inline Array2s& max() { return m_max; }

  inline const Array2s& min() const { return m_min; }
  inline const Array2s& max() const { return m_max; }

private:

  Array2s m_min;
  Array2s m_max;

};

namespace SpatialGridDetector
{
  void getPotentialOverlaps( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
  void getPotentialOverlapsAllPairs( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
}

#endif
