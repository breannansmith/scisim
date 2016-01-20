// ConstraintCache.cpp
//
// Breannan Smith
// Last updated: 01/18/2016

#include "ConstraintCache.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/Utilities.h"

#include <iostream>

// TODO: Kinemtic-circle should be separate, as it has different invariants

void ConstraintCache::cacheConstraint( const Constraint& constraint, const VectorXs& r )
{
  const std::string constraint_type{ constraint.name() };

  if( constraint_type == "circle_circle" )
  {
    assert( constraint.simulatedBody0() < constraint.simulatedBody1() );
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    m_circle_circle_constraints.insert( std::make_pair( std::make_pair( constraint.simulatedBody0(), constraint.simulatedBody1() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else if( constraint_type == "static_plane_circle" )
  {
    assert( constraint.simulatedBody1() == -1 );
    #ifndef NDEBUG
    const auto insert_return =
    #endif
    m_plane_circle_constraints.insert( std::make_pair( std::make_pair( constraint.getStaticObjectIndex(), constraint.simulatedBody0() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else
  {
    std::cerr << constraint.name() << " not supported in ConstraintCache::cacheConstraint. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}

void ConstraintCache::clear()
{
  m_circle_circle_constraints.clear();
  m_plane_circle_constraints.clear();
}

bool ConstraintCache::empty() const
{
  return m_circle_circle_constraints.empty() && m_plane_circle_constraints.empty();
}

void ConstraintCache::getCachedConstraint( const Constraint& constraint, VectorXs& r ) const
{
  const std::string constraint_type{ constraint.name() };

  if( constraint_type == "circle_circle" )
  {
    assert( constraint.simulatedBody0() < constraint.simulatedBody1() );
    const auto map_iterator = m_circle_circle_constraints.find( std::make_pair( constraint.simulatedBody0(), constraint.simulatedBody1() ) );
    if( map_iterator != m_circle_circle_constraints.end() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else if( constraint_type == "static_plane_circle" )
  {
    assert( constraint.simulatedBody1() == -1 );
    const auto map_iterator = m_plane_circle_constraints.find( std::make_pair( constraint.getStaticObjectIndex(), constraint.simulatedBody0() ) );
    if( map_iterator != m_plane_circle_constraints.end() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else
  {
    std::cerr << constraint_type << " not supported in ConstraintCache::getCachedConstraint. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // If the constraint was not found set to a default force of 0
  r.setZero();
}

static void serializeCache( const std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::ostream& output_stream )
{
  assert( output_stream.good() );
  Utilities::serializeBuiltInType( constraint_cache.size(), output_stream );
  for( const std::pair<std::pair<unsigned,unsigned>,VectorXs>& con : constraint_cache )
  {
    Utilities::serializeBuiltInType( con.first.first, output_stream );
    Utilities::serializeBuiltInType( con.first.second, output_stream );
    MathUtilities::serialize( con.second, output_stream );
  }
}

static void deserializeCache( std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::istream& input_stream )
{
  assert( input_stream.good() );
  using map_size_t = std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type;
  const map_size_t ncons{ Utilities::deserialize<map_size_t>( input_stream ) };
  for( unsigned con_num = 0; con_num < ncons; ++con_num )
  {
    const unsigned first_index{ Utilities::deserialize<unsigned>( input_stream ) };
    const unsigned second_index{ Utilities::deserialize<unsigned>( input_stream ) };
    const VectorXs force{ MathUtilities::deserialize<VectorXs>( input_stream ) };
    std::pair< std::map< std::pair<unsigned,unsigned>, VectorXs >::iterator, bool > insert_return;
    insert_return = constraint_cache.insert( std::make_pair( std::make_pair( first_index, second_index ), force ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
}

void ConstraintCache::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  serializeCache( m_circle_circle_constraints, output_stream );
  serializeCache( m_plane_circle_constraints, output_stream );
}

void ConstraintCache::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_circle_circle_constraints.clear();
  deserializeCache( m_circle_circle_constraints, input_stream );
  m_plane_circle_constraints.clear();
  deserializeCache( m_plane_circle_constraints, input_stream );
  // TODO: Check that all dimensions of forces are the same
}
