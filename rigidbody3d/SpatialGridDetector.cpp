// SpatialGridDetector.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "SpatialGridDetector.h"

bool AABB::overlaps( const AABB& other ) const
{
  // Temporary sanity check: internal code shouldn't compare an AABB to itself
  assert( &other != this );

  // Attempt to find a separating axis
  if( ( this->max() < other.min() ).any() )
  {
    return false;
  }
  if( ( other.max() < this->min() ).any() )
  {
    return false;
  }

  // If no separating axis exists, the AABBs overlap
  return true;
}



static void computeCellIndex( const Array3s& coord, const Array3s& min_coord, const scalar& h, Array3u& index )
{
  assert( ( coord > min_coord ).all() ); assert( h > 0.0 );
  // Unsigned cast same as floor if input is positive
  index = ( ( coord - min_coord ) / h ).cast<unsigned>();
}

static unsigned keyForIndex( const Array3u& index, const Array3u& dimensions )
{
  assert( ( index < dimensions ).all() );
  // TODO: key could get really big...
  return index.x() + dimensions.x() * index.y() + dimensions.x() * dimensions.y() * index.z();
}

static void rasterizeAABBs( const std::vector<AABB>& aabbs, const Array3s& min_coord, const scalar& h, const Array3u& dimensions, std::map<unsigned,std::vector<unsigned>>& voxels )
{
  // For each bounding box
  for( std::vector<AABB>::size_type aabb_idx = 0; aabb_idx < aabbs.size(); ++aabb_idx )
  {
    // Compute the cells the AABB overlaps with. Slightly enlarge the boxes to account for FPA errors.
    Array3u index_lower;
    computeCellIndex( aabbs[aabb_idx].min() - 1.0e-6, min_coord, h, index_lower );
    Array3u index_upper;
    computeCellIndex( aabbs[aabb_idx].max() + 1.0e-6, min_coord, h, index_upper );
    assert( ( index_lower <= index_upper ).all() );

    for( unsigned x_idx = index_lower.x(); x_idx <= index_upper.x(); ++x_idx )
    {
      for( unsigned y_idx = index_lower.y(); y_idx <= index_upper.y(); ++y_idx )
      {
        for( unsigned z_idx = index_lower.z(); z_idx <= index_upper.z(); ++z_idx )
        {
          // Compute the hash key for the given indexed voxel
          const unsigned key = keyForIndex( Array3u{ x_idx, y_idx, z_idx }, dimensions );

          auto voxel_iterator = voxels.find( key );
          // Create a new voxel, if needed
          if( voxel_iterator == voxels.end() )
          {
            const auto insertion_result = voxels.insert( std::pair<unsigned,std::vector<unsigned>>( key, std::vector<unsigned>{} ) );
            assert( insertion_result.second );
            voxel_iterator = insertion_result.first;
          }
          // Add the index of the AABB to the voxel
          voxel_iterator->second.emplace_back( unsigned( aabb_idx ) );
        }
      }
    }
  }
}

static void initializeSpatialGrid( const std::vector<AABB>& aabbs, Array3s& min_coord, Array3u& dimensions, scalar& h )
{
  // Compute a bounding box for all AABBs
  min_coord = Array3s::Constant( SCALAR_INFINITY );
  Array3s max_coord{ Array3s::Constant( -SCALAR_INFINITY ) };
  for( std::vector<AABB>::size_type aabb_idx = 0; aabb_idx < aabbs.size(); ++aabb_idx )
  {
    assert( ( aabbs[aabb_idx].min() < aabbs[aabb_idx].max() ).all() );
    min_coord = min_coord.min( aabbs[aabb_idx].min() );
    max_coord = max_coord.max( aabbs[aabb_idx].max() );
    assert( ( min_coord < max_coord ).all() );
  }
  // Inflate the AABB to account for FPA quantization errors
  min_coord -= 2.0e-6;
  max_coord += 2.0e-6;

  // Compute the grid cell width
  {
    Array3s delta{ Array3s::Zero() };
    for( std::vector<AABB>::size_type aabb_idx = 0; aabb_idx < aabbs.size(); ++aabb_idx )
    {
      delta += aabbs[aabb_idx].max() - aabbs[aabb_idx].min();
    }
    h = delta.maxCoeff() / scalar( aabbs.size() );
  }

  // Compute the number of cells in the grid
  dimensions = ( ( max_coord - min_coord ) / h ).unaryExpr(std::ptr_fun(ceil)).cast<unsigned>();
}

void SpatialGridDetector::getPotentialOverlaps( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps )
{
  Array3s min_coord;
  Array3u dimensions;
  scalar h;
  initializeSpatialGrid( aabbs, min_coord, dimensions, h );

  std::map<unsigned,std::vector<unsigned>> voxels;
  rasterizeAABBs( aabbs, min_coord, h, dimensions, voxels );

  // For each voxel
  for( auto itr = voxels.cbegin(); itr != voxels.cend(); ++itr )
  {
    // Visit each pair of AABBs in this voxel
    for( std::vector<unsigned>::size_type idx0 = 0; idx0 + 1 < (*itr).second.size(); ++idx0 )
    {
      for( std::vector<unsigned>::size_type idx1 = idx0 + 1; idx1 < (*itr).second.size(); ++idx1 )
      {
        // If the AABBs overlap
        if( aabbs[(*itr).second[idx0]].overlaps( aabbs[(*itr).second[idx1]] ) )
        {
          assert( (*itr).second[idx0] < (*itr).second[idx1] );
          overlaps.insert( std::make_pair( (*itr).second[idx0], (*itr).second[idx1] ) );
        }
      }
    }
  }
}

void SpatialGridDetector::getPotentialOverlapsAllPairs( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps )
{
  for( std::vector<AABB>::size_type idx0 = 0; idx0 < aabbs.size(); ++idx0 )
  {
    for( std::vector<AABB>::size_type idx1 = idx0 + 1; idx1 < aabbs.size(); ++idx1 )
    {
      if( aabbs[idx0].overlaps( aabbs[idx1] ) )
      {
        overlaps.insert( std::pair<unsigned,unsigned>( idx0, idx1 ) );
      }
    }
  }
}
