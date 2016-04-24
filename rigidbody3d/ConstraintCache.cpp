// ConstraintCache.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "ConstraintCache.h"

#include <iostream>

#include "scisim/Math/MathUtilities.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/Utilities.h"

#include "Constraints/SphereSphereConstraint.h"
#include "Constraints/StaticPlaneSphereConstraint.h"
#include "Constraints/StaticCylinderSphereConstraint.h"

void ConstraintCache::cacheConstraint( const Constraint& constraint, const VectorXs& r )
{
  if( constraint.name() == "sphere_sphere" )
  {
    const SphereSphereConstraint& sphere_sphere{ static_cast<const SphereSphereConstraint&>( constraint ) };
    // Insert this constraint into the cache
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    m_sphere_sphere_constraint_cache.insert( std::make_pair( std::make_pair( sphere_sphere.idx0(), sphere_sphere.idx1() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else if( constraint.name() == "static_plane_sphere" )
  {
    const StaticPlaneSphereConstraint& plane_sphere{ static_cast<const StaticPlaneSphereConstraint&>( constraint ) };
    // Insert this constraint into the cache
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    m_static_plane_sphere_constraint_cache.insert( std::make_pair( std::make_pair( plane_sphere.planeIdx(), plane_sphere.sphereIdx() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else if( constraint.name() == "static_cylinder_sphere" )
  {
    const StaticCylinderSphereConstraint& cylinder_sphere{ static_cast<const StaticCylinderSphereConstraint&>( constraint ) };
    // Insert this constraint into the cache
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    m_static_cylinder_sphere_constraint_cache.insert( std::make_pair( std::make_pair( cylinder_sphere.cylinderIdx(), cylinder_sphere.sphereIdx() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else
  {
    // Unhandled constraint, warn the user!
    std::cerr << constraint.name() << " not supported in ConstraintCache::cacheConstraint, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}

void ConstraintCache::clear()
{
  m_sphere_sphere_constraint_cache.clear();
  m_box_sphere_constraint_cache.clear();
  m_static_plane_sphere_constraint_cache.clear();
  m_static_cylinder_sphere_constraint_cache.clear();
}

bool ConstraintCache::empty() const
{
  return m_sphere_sphere_constraint_cache.empty() && m_box_sphere_constraint_cache.empty() && m_static_plane_sphere_constraint_cache.empty() && m_static_cylinder_sphere_constraint_cache.empty();
}

void ConstraintCache::getCachedConstraint( const Constraint& constraint, VectorXs& r ) const
{
  if( constraint.name() == "sphere_sphere" )
  {
    const SphereSphereConstraint& sphere_sphere{ static_cast<const SphereSphereConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    using itr_type = std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator;
    const itr_type map_iterator{ m_sphere_sphere_constraint_cache.find( std::make_pair( sphere_sphere.idx0(), sphere_sphere.idx1() ) ) };
    // If object is in the cache, return the force
    if( map_iterator != m_sphere_sphere_constraint_cache.cend() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else if( constraint.name() == "static_plane_sphere" )
  {
    const StaticPlaneSphereConstraint& plane_sphere{ static_cast<const StaticPlaneSphereConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    using itr_type = std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator;
    const itr_type map_iterator{ m_static_plane_sphere_constraint_cache.find( std::make_pair( plane_sphere.planeIdx(), plane_sphere.sphereIdx() ) ) };
    // If object is in the cache, return the force
    if( map_iterator != m_static_plane_sphere_constraint_cache.cend() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else if( constraint.name() == "static_cylinder_sphere" )
  {
    const StaticCylinderSphereConstraint& cylinder_sphere{ static_cast<const StaticCylinderSphereConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    using itr_type = std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator;
    const itr_type map_iterator{ m_static_cylinder_sphere_constraint_cache.find( std::make_pair( cylinder_sphere.cylinderIdx(), cylinder_sphere.sphereIdx() ) ) };
    // If object is in the cache, return the force
    if( map_iterator != m_static_cylinder_sphere_constraint_cache.cend() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else
  {
    std::cerr << constraint.name() << " not supported in ConstraintCache::getCachedConstraint, exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // If the constraint was not found, return 0
  r.setZero();
}

static void serializeCache( const std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::ostream& output_stream )
{
  assert( output_stream.good() );
  Utilities::serialize( constraint_cache.size(), output_stream );
  // For some reason, clang static analyzer does not like the range based for loop here
  for( auto iterator = constraint_cache.cbegin(); iterator != constraint_cache.cend(); ++iterator )
  {
    Utilities::serialize( iterator->first.first, output_stream );
    Utilities::serialize( iterator->first.second, output_stream );
    MathUtilities::serialize( iterator->second, output_stream );
  }
}

void ConstraintCache::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  serializeCache( m_sphere_sphere_constraint_cache, output_stream );
  serializeCache( m_box_sphere_constraint_cache, output_stream );
  serializeCache( m_static_plane_sphere_constraint_cache, output_stream );
  serializeCache( m_static_cylinder_sphere_constraint_cache, output_stream );
}

static void deserializeCache( std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::istream& input_stream )
{
  assert( input_stream.good() );
  const std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type ncons{ Utilities::deserialize<std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type>( input_stream ) };
  for( unsigned con_num = 0; con_num < ncons; ++con_num )
  {
    const unsigned first_index{ Utilities::deserialize<unsigned>( input_stream ) };
    const unsigned second_index{ Utilities::deserialize<unsigned>( input_stream ) };
    const VectorXs force{ MathUtilities::deserialize<VectorXs>( input_stream ) };
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    constraint_cache.insert( std::make_pair( std::make_pair( first_index, second_index ), force ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
}

void ConstraintCache::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_sphere_sphere_constraint_cache.clear();
  deserializeCache( m_sphere_sphere_constraint_cache, input_stream );
  m_box_sphere_constraint_cache.clear();
  deserializeCache( m_box_sphere_constraint_cache, input_stream );
  m_static_plane_sphere_constraint_cache.clear();
  deserializeCache( m_static_plane_sphere_constraint_cache, input_stream );
  m_static_cylinder_sphere_constraint_cache.clear();
  deserializeCache( m_static_cylinder_sphere_constraint_cache, input_stream );
}
