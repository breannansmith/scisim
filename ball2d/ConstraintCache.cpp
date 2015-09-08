// ConstraintCache.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "ConstraintCache.h"

#include "SCISim/Math/MathUtilities.h"
#include "SCISim/Constraints/Constraint.h"
#include "SCISim/Utilities.h"
#include "Constraints/BallBallConstraint.h"
#include "Constraints/BallStaticPlaneConstraint.h"
#include "Constraints/BallStaticDrumConstraint.h"

#include <iostream>

void ConstraintCache::cacheConstraint( const Constraint& constraint, const VectorXs& r )
{
  if( constraint.name() == "ball_ball" )
  {
    const BallBallConstraint& ball_ball{ sd_cast<const BallBallConstraint&>( constraint ) };
    // Insert this constraint into the cache
    std::pair<std::map<std::pair<unsigned,unsigned>,VectorXs>::iterator,bool> insert_return;
    insert_return = m_ball_ball_constraints.insert( std::make_pair( std::make_pair( ball_ball.idx0(), ball_ball.idx1() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else if( constraint.name() == "static_plane_constraint" )
  {
    const StaticPlaneConstraint& plane_ball{ sd_cast<const StaticPlaneConstraint&>( constraint ) };
    // Insert this constraint into the cache
    std::pair<std::map<std::pair<unsigned,unsigned>,VectorXs>::iterator,bool> insert_return;
    insert_return = m_plane_ball_constraints.insert( std::make_pair( std::make_pair( plane_ball.planeIdx(), plane_ball.ballIdx() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else if( constraint.name() == "static_drum_constraint" )
  {
    const StaticDrumConstraint& drum_ball{ sd_cast<const StaticDrumConstraint&>( constraint ) };
    // Insert this constraint into the cache
    std::pair<std::map<std::pair<unsigned,unsigned>,VectorXs>::iterator,bool> insert_return;
    insert_return = m_drum_ball_constraints.insert( std::make_pair( std::make_pair( drum_ball.drumIdx(), drum_ball.ballIdx() ), r ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
  else
  {
    std::cerr << constraint.name() << " not supported in ConstraintCache::cacheConstraint, this is a bug. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}

void ConstraintCache::clear()
{
  m_ball_ball_constraints.clear();
  m_plane_ball_constraints.clear();
  m_drum_ball_constraints.clear();
}

void ConstraintCache::getCachedConstraint( const Constraint& constraint, VectorXs& r ) const
{
  if( constraint.name() == "ball_ball" )
  {
    const BallBallConstraint& ball_ball{ sd_cast<const BallBallConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator map_iterator;
    map_iterator = m_ball_ball_constraints.find( std::make_pair( ball_ball.idx0(), ball_ball.idx1() ) );
    // If object is in the cache, retrieve the force
    if( map_iterator != m_ball_ball_constraints.end() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else if( constraint.name() == "static_plane_constraint" )
  {
    const StaticPlaneConstraint& plane_ball{ sd_cast<const StaticPlaneConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator map_iterator;
    map_iterator = m_plane_ball_constraints.find( std::make_pair( plane_ball.planeIdx(), plane_ball.ballIdx() ) );
    // If object is in the cache, retrieve the force
    if( map_iterator != m_plane_ball_constraints.end() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else if( constraint.name() == "static_drum_constraint" )
  {
    const StaticDrumConstraint& drum_ball{ sd_cast<const StaticDrumConstraint&>( constraint ) };
    // Try to retrieve this constraint from the cache
    std::map<std::pair<unsigned,unsigned>,VectorXs>::const_iterator map_iterator;
    map_iterator = m_drum_ball_constraints.find( std::make_pair( drum_ball.drumIdx(), drum_ball.ballIdx() ) );
    // If object is in the cache, return the force
    if( map_iterator != m_drum_ball_constraints.end() )
    {
      assert( r.size() == map_iterator->second.size() );
      r = map_iterator->second;
      return;
    }
  }
  else
  {
    std::cerr << constraint.name() << " not supported in ConstraintCache::getCachedConstraint, this is a bug. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // If the constraint was not found set to a default force of 0
  r.setZero();
}

// TODO: Use Utilities serialization routines
static void serializeCache( const std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::ostream& output_stream )
{
  assert( output_stream.good() );
  {
    std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type ncons{ constraint_cache.size() };
    output_stream.write( reinterpret_cast<char*>( &ncons ), sizeof(std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type) );
  }
  for( const std::pair<std::pair<unsigned,unsigned>,VectorXs>& con : constraint_cache )
  {
    {
      unsigned first_index{ con.first.first };
      output_stream.write( reinterpret_cast<char*>( &first_index ), sizeof(unsigned) );
    }
    {
      unsigned second_index{ con.first.second };
      output_stream.write( reinterpret_cast<char*>( &second_index ), sizeof(unsigned) );
    }
    {
      const VectorXs& force{ con.second };
      MathUtilities::serialize( force, output_stream );
    }
  }
}

// TODO: Use Utilities deserialization routines
static void deserializeCache( std::map<std::pair<unsigned,unsigned>,VectorXs>& constraint_cache, std::istream& input_stream )
{
  assert( input_stream.good() );
  std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type ncons;
  input_stream.read( (char*) &ncons, sizeof(std::map<std::pair<unsigned,unsigned>,VectorXs>::size_type) );
  for( unsigned con_num = 0; con_num < ncons; ++con_num )
  {
    unsigned first_index;
    input_stream.read( reinterpret_cast<char*>( &first_index ), sizeof(unsigned) );
    unsigned second_index;
    input_stream.read( reinterpret_cast<char*>( &second_index ), sizeof(unsigned) );
    const VectorXs force = MathUtilities::deserialize<VectorXs>( input_stream );
    std::pair< std::map< std::pair<unsigned,unsigned>, VectorXs >::iterator, bool > insert_return;
    insert_return = constraint_cache.insert( std::make_pair( std::make_pair( first_index, second_index ), force ) );
    assert( insert_return.second ); // Should not re-encounter constraints
  }
}

void ConstraintCache::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  serializeCache( m_ball_ball_constraints, output_stream );
  serializeCache( m_plane_ball_constraints, output_stream );
  serializeCache( m_drum_ball_constraints, output_stream );
}

void ConstraintCache::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_ball_ball_constraints.clear();
  deserializeCache( m_ball_ball_constraints, input_stream );
  m_plane_ball_constraints.clear();
  deserializeCache( m_plane_ball_constraints, input_stream );
  m_drum_ball_constraints.clear();
  deserializeCache( m_drum_ball_constraints, input_stream );
  // TODO: Check that all dimensions of forces are the same
}
