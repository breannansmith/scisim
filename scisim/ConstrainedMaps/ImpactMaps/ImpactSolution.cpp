// ImpactSolution.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "ImpactSolution.h"

#include "scisim/Constraints/Constraint.h"
#include "scisim/HDF5File.h"

ImpactSolution::ImpactSolution()
: m_indices()
, m_points()
, m_normals()
, m_forces()
, m_dt()
{}

static void getCollisionIndices( const Constraint& con, std::pair<int,int>& indices )
{
  con.getBodyIndices( indices );
  if( indices.second == -1 )
  {
    const unsigned static_object_index{ con.getStaticObjectIndex() };
    indices.second = - int( static_object_index ) - 2;
  }
}

void ImpactSolution::setSolution( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& impact_bases, const VectorXs& alpha, const scalar& dt )
{
  const unsigned ncons{ static_cast<unsigned>( constraints.size() ) };
  assert( ncons == alpha.size() );
  assert( std::vector<std::unique_ptr<Constraint>>::size_type( ncons ) == constraints.size() );
  assert( alpha.size() == ncons );

  const unsigned ambient_space_dims{ static_cast<unsigned>( impact_bases.rows() ) };
  assert( ambient_space_dims == 2 || ambient_space_dims == 3 );

  // Place all indices into a single matrix for output
  m_indices.resize( 2, ncons );
  for( unsigned con = 0; con < ncons; ++con )
  {
    std::pair<int,int> indices;
    assert( constraints[con] != nullptr );
    getCollisionIndices( *constraints[con], indices );
    m_indices.col( con ) << indices.first, indices.second;
  }

  // Save the world space contact points
  m_points.resize( ambient_space_dims, ncons );
  for( unsigned con = 0; con < ncons; ++con )
  {
    VectorXs contact_point;
    assert( constraints[con] != nullptr );
    constraints[con]->getWorldSpaceContactPoint( q, contact_point );
    assert( contact_point.size() == ambient_space_dims );
    m_points.col( con ) = contact_point;
  }

  // Save the world space contact normals
  m_normals = impact_bases;

  // Compute the world space contact forces
  m_forces.resize( ambient_space_dims, ncons );
  for( unsigned con = 0; con < ncons; ++con )
  {
    // Compute the contact force
    m_forces.col( con ) = alpha( con ) * m_normals.col( con );
  }

  // Save out the time step (useful in certain bits of analysis)
  m_dt = dt;
}

void ImpactSolution::writeSolution( HDF5File& output_file )
{
  const unsigned ncons{ unsigned( m_indices.cols() ) };

  output_file.writeScalar( "collision_count", ncons );

  // NB: Prior to version 1.8.7, HDF5 did not support zero sized dimensions.
  //     Some versions of Ubuntu still have old versions of HDF5, so workaround.
  // TODO: Remove once our servers are updated.
  if( ncons == 0 )
  {
    return;
  }

  assert( m_points.rows() == 2 || m_points.rows() == 3 );
  assert( m_points.cols() == m_indices.cols() );
  assert( m_points.rows() == m_normals.rows() );
  assert( m_points.cols() == m_normals.cols() );
  assert( m_points.rows() == m_forces.rows() );
  assert( m_points.cols() == m_forces.cols() );

  output_file.writeMatrix( "collision_indices", m_indices );
  output_file.writeMatrix( "collision_points", m_points );
  output_file.writeMatrix( "collision_normals", m_normals );
  output_file.writeMatrix( "collision_forces", m_forces );
}
