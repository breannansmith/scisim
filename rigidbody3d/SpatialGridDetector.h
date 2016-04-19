// SpatialGridDetector.h
//
// Breannan Smith
// Last updated: 09/14/2015

#ifndef SPATIAL_GRID_DETECTOR_H
#define SPATIAL_GRID_DETECTOR_H

#include "scisim/Math/MathDefines.h"

#include <set>
#include <vector>

class AABB final
{

public:

  bool overlaps( const AABB& other ) const;

  // TODO: Remove non-const accessors, replace with constructor
  inline Array3s& min() { return m_min; }
  inline Array3s& max() { return m_max; }

  inline const Array3s& min() const { return m_min; }
  inline const Array3s& max() const { return m_max; }

private:  

  Array3s m_min;
  Array3s m_max;

};

namespace SpatialGridDetector
{
  void getPotentialOverlaps( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
  void getPotentialOverlapsAllPairs( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
}

#endif
