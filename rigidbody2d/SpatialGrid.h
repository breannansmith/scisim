// SpatialGridDetector.h
//
// Breannan Smith
// Last updated: 09/10/2015

#ifndef SPATIAL_GRID_H
#define SPATIAL_GRID_H

#include "scisim/Math/MathDefines.h"

#include <set>
#include <vector>

class AABB final
{

public:

  AABB( const Array2s& min, const Array2s& max );

  bool overlaps( const AABB& other ) const;

  inline const Array2s& min() const { return m_min; }
  inline const Array2s& max() const { return m_max; }

private:

  Array2s m_min;
  Array2s m_max;

};

namespace SpatialGrid
{
  void getPotentialOverlaps( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
  // TODO: Can make this faster by actually using the grid
  // TODO: Create a version that caches the grid for multiple lookups
  void getPotentialOverlaps( const AABB& trial_aabb, const std::vector<AABB>& aabbs, std::vector<unsigned>& overlaps );
}

#endif
