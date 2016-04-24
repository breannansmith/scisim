// RigidBody3DState.cpp
//
// Breannan Smith
// Last updated: 09/24/2015

#include "RigidBody3DState.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/StringUtilities.h"
#include "scisim/Utilities.h"

#include "Geometry/RigidBodyBox.h"
#include "Geometry/RigidBodySphere.h"
#include "Geometry/RigidBodyTriangleMesh.h"

#include "Forces/NearEarthGravityForce.h"

#include "StaticGeometry/StaticPlane.h"

#include <iostream>

RigidBody3DState::RigidBody3DState()
: m_nbodies( 0 )
, m_q()
, m_v()
, m_M0()
, m_Minv0()
, m_M()
, m_Minv()
, m_fixed()
, m_geometry()
, m_geometry_indices()
, m_forces()
, m_static_planes()
, m_static_cylinders()
, m_planar_portals()
{}

RigidBody3DState::RigidBody3DState( const RigidBody3DState& other )
: m_nbodies( other.m_nbodies )
, m_q( other.m_q )
, m_v( other.m_v )
, m_M0( other.m_M0 )
, m_Minv0( other.m_Minv0 )
, m_M( other.m_M )
, m_Minv( other.m_Minv )
, m_fixed( other.m_fixed )
, m_geometry( Utilities::clone( other.m_geometry ) )
, m_geometry_indices( other.m_geometry_indices )
, m_forces( Utilities::clone( other.m_forces ) )
, m_static_planes( other.m_static_planes )
, m_static_cylinders( other.m_static_cylinders )
, m_planar_portals( other.m_planar_portals )
{}

RigidBody3DState& RigidBody3DState::operator=( const RigidBody3DState& other )
{
  RigidBody3DState copy{ other };
  using std::swap;
  swap( *this, copy );
  return *this;
}

static SparseMatrixsc formBodySpaceMassMatrix( const std::vector<scalar>& M, const std::vector<Vector3s>& I0 )
{
  assert( M.size() == I0.size() );
  const unsigned nbodies{ static_cast<unsigned>( I0.size() ) };
  const unsigned nvdofs{ 6 * nbodies };

  SparseMatrixsc M0{ static_cast<SparseMatrixsc::Index>( nvdofs ), static_cast<SparseMatrixsc::Index>( nvdofs ) };
  M0.reserve( VectorXi::Ones( nvdofs ) );
  // Load the total masses
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( M[ bdy_idx ] > 0.0 );
      M0.insert( row, col ) = M[ bdy_idx ];
    }
  }
  // Load the inertia tensors
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * nbodies + 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( I0[ bdy_idx ]( dof_idx ) > 0.0 );
      M0.insert( row, col ) = I0[ bdy_idx ]( dof_idx );
    }
  }
  assert( nvdofs == unsigned( M0.nonZeros() ) );
  M0.makeCompressed();
  return M0;
}

static SparseMatrixsc formBodySpaceInverseMassMatrix( const std::vector<scalar>& M, const std::vector<Vector3s>& I0 )
{
  assert( M.size() == I0.size() );
  const unsigned nbodies{ static_cast<unsigned>( I0.size() ) };
  const unsigned nvdofs{ 6 * nbodies };

  SparseMatrixsc M0{ static_cast<SparseMatrixsc::Index>( nvdofs ), static_cast<SparseMatrixsc::Index>( nvdofs ) };
  M0.reserve( VectorXi::Ones( nvdofs ) );
  // Load the total masses
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( M[ bdy_idx ] > 0.0 );
      M0.insert( row, col ) = 1.0 / M[ bdy_idx ];
    }
  }
  // Load the inertia tensors
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * nbodies + 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( I0[ bdy_idx ]( dof_idx ) > 0.0 );
      M0.insert( row, col ) = 1.0 / I0[ bdy_idx ]( dof_idx );
    }
  }
  assert( nvdofs == unsigned( M0.nonZeros() ) );
  M0.makeCompressed();
  return M0;
}

static SparseMatrixsc formWorldSpaceMassMatrix( const std::vector<scalar>& M, const std::vector<Vector3s>& I0, const std::vector<VectorXs>& R )
{
  assert( M.size() == I0.size() );
  assert( M.size() == I0.size() );
  const unsigned nbodies{ static_cast<unsigned>( I0.size() ) };
  const unsigned nvdofs{ 6 * nbodies };

  SparseMatrixsc Mbody{ static_cast<SparseMatrixsc::Index>( nvdofs ), static_cast<SparseMatrixsc::Index>( nvdofs ) };
  {
    VectorXi column_nonzeros{ nvdofs };
    column_nonzeros.segment( 0, 3 * nbodies ).setOnes();
    column_nonzeros.segment( 3 * nbodies, 3 * nbodies ).setConstant( 3 );
    Mbody.reserve( column_nonzeros );
  }
  // Load the total masses
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( M[ bdy_idx ] > 0.0 );
      Mbody.insert( row, col ) = M[ bdy_idx ];
    }
  }

  // Load the inertia tensors
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    // Transform from principal axes rep
    const Eigen::Map<const Matrix33sr> Rmat{ R[bdy_idx].data() };
    assert( ( Rmat * Rmat.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    assert( fabs( Rmat.determinant() - 1.0 ) <= 1.0e-9 );
    const Matrix33sr I = Rmat * I0[bdy_idx].asDiagonal() * Rmat.transpose();
    assert( ( I - I.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-12 );
    assert( I.determinant() > 0.0 );
    for( unsigned row_idx = 0; row_idx < 3; ++row_idx )
    {
      const unsigned col{ 3 * nbodies + 3 * bdy_idx + row_idx };
      for( unsigned col_idx = 0; col_idx < 3; ++col_idx )
      {
        const unsigned row{ 3 * nbodies + 3 * bdy_idx + col_idx };
        Mbody.insert( row, col ) = I( row_idx, col_idx );
      }
    }
  }
  assert( 12 * nbodies == unsigned( Mbody.nonZeros() ) );
  Mbody.makeCompressed();
  return Mbody;
}

static SparseMatrixsc formWorldSpaceInverseMassMatrix( const std::vector<scalar>& M, const std::vector<Vector3s>& I0, const std::vector<VectorXs>& R )
{
  assert( M.size() == I0.size() );
  assert( M.size() == I0.size() );
  const unsigned nbodies{ static_cast<unsigned>( I0.size() ) };
  const unsigned nvdofs{ 6 * nbodies };

  SparseMatrixsc Mbody{ static_cast<SparseMatrixsc::Index>( nvdofs ), static_cast<SparseMatrixsc::Index>( nvdofs ) };
  {
    VectorXi column_nonzeros{ nvdofs };
    column_nonzeros.segment( 0, 3 * nbodies ).setOnes();
    column_nonzeros.segment( 3 * nbodies, 3 * nbodies ).setConstant( 3 );
    Mbody.reserve( column_nonzeros );
  }
  // Load the total masses
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    for( unsigned dof_idx = 0; dof_idx < 3; ++dof_idx )
    {
      const unsigned col{ 3 * bdy_idx + dof_idx };
      const unsigned row{ col };
      assert( M[ bdy_idx ] > 0.0 );
      Mbody.insert( row, col ) = 1.0 / M[ bdy_idx ];
    }
  }

  // Load the inertia tensors
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    // Transform from principal axes rep
    const Eigen::Map<const Matrix33sr> Rmat{ R[bdy_idx].data() };
    assert( ( Rmat * Rmat.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    assert( fabs( Rmat.determinant() - 1.0 ) <= 1.0e-9 );
    const Matrix33sr Iinv = Rmat * I0[bdy_idx].array().inverse().matrix().asDiagonal() * Rmat.transpose();
    assert( ( Iinv - Iinv.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-12 );
    assert( Iinv.determinant() > 0.0 );
    for( unsigned row_idx = 0; row_idx < 3; ++row_idx )
    {
      const unsigned col{ 3 * nbodies + 3 * bdy_idx + row_idx };
      for( unsigned col_idx = 0; col_idx < 3; ++col_idx )
      {
        const unsigned row{ 3 * nbodies + 3 * bdy_idx + col_idx };
        Mbody.insert( row, col ) = Iinv( row_idx, col_idx );
      }
    }
  }
  assert( 12 * nbodies == unsigned( Mbody.nonZeros() ) );
  Mbody.makeCompressed();
  return Mbody;
}

void RigidBody3DState::setState( const std::vector<Vector3s>& X, const std::vector<Vector3s>& V, const std::vector<scalar>& M, const std::vector<VectorXs>& R, const std::vector<Vector3s>& omega, const std::vector<Vector3s>& I0, const std::vector<bool>& fixed, const std::vector<unsigned>& geom_indices, const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry )
{
  assert( X.size() == V.size() );
  assert( X.size() == M.size() );
  assert( X.size() == R.size() );
  assert( X.size() == omega.size() );
  assert( X.size() == I0.size() );
  assert( X.size() == fixed.size() );
  assert( X.size() == geom_indices.size() );

  m_nbodies = unsigned( X.size() );

  // Initialize q
  const unsigned nqdofs{ 12 * unsigned( X.size() ) };
  m_q.resize( nqdofs );
  // Load in center of mass positions
  for( std::vector<Vector3s>::size_type body_num = 0; body_num < X.size(); ++body_num )
  {
    m_q.segment<3>( 3 * body_num ) = X[body_num];
  }
  // Load in the orientations
  for( std::vector<VectorXs>::size_type body_num = 0; body_num < R.size(); ++body_num )
  {
    m_q.segment<9>( 3 * m_nbodies + 9 * body_num ) = R[body_num];
    #ifndef NDEBUG
    {
      const Eigen::Map<const Matrix33sr> Rmat{ R[body_num].data() };
      assert( ( Rmat * Rmat.transpose()- Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() < 1.0e-9 );
      assert( fabs( Rmat.determinant() - 1.0 ) <= 1.0e-9 );
    }
    #endif
  }

  // Initialize v
  const unsigned nvdofs{ 6 * unsigned( omega.size() ) };
  m_v.resize( nvdofs );
  // Load in center mass velocities
  for( std::vector<Vector3s>::size_type body_num = 0; body_num < V.size(); ++body_num )
  {
    m_v.segment<3>( 3 * body_num ) = V[body_num];
  }
  // Load in angular velocities
  for( std::vector<Vector3s>::size_type body_num = 0; body_num < omega.size(); ++body_num )
  {
    m_v.segment<3>( 3 * m_nbodies + 3 * body_num ) = omega[body_num];
  }

  // Load the mass matrices
  if( nvdofs > 0 )
  {
    m_M0 = formBodySpaceMassMatrix( M, I0 );
    m_Minv0 = formBodySpaceInverseMassMatrix( M, I0 );
    m_M = formWorldSpaceMassMatrix( M, I0, R );
    m_Minv = formWorldSpaceInverseMassMatrix( M, I0, R );
  }

  assert( MathUtilities::isIdentity( m_M0 * m_Minv0, 1.0e-9 ) );
  assert( MathUtilities::isIdentity( m_M * m_Minv, 1.0e-9 ) );

  m_fixed = fixed;
  m_geometry_indices = geom_indices;

  // Load the geometry
  m_geometry = Utilities::clone( geometry );
  assert( std::all_of( m_geometry.cbegin(), m_geometry.cend(), []( const auto& geo ) { return geo != nullptr; } ) );
  assert( std::all_of( m_geometry_indices.cbegin(), m_geometry_indices.cend(), [this]( const auto idx ) { return idx < m_geometry.size(); } ) );
}

unsigned RigidBody3DState::nbodies() const
{
  return m_nbodies;
}

unsigned RigidBody3DState::ngeo() const
{
  return unsigned( m_geometry.size() );
}

VectorXs& RigidBody3DState::q()
{
  return m_q;
}

const VectorXs& RigidBody3DState::q() const
{
  return m_q;
}

VectorXs& RigidBody3DState::v()
{
  return m_v;
}

const VectorXs& RigidBody3DState::v() const
{
  return m_v;
}

SparseMatrixsc& RigidBody3DState::M0()
{
  return m_M0;
}

const SparseMatrixsc& RigidBody3DState::M0() const
{
  return m_M0;
}

SparseMatrixsc& RigidBody3DState::M()
{
  return m_M;
}

const SparseMatrixsc& RigidBody3DState::M() const
{
  return m_M;
}

SparseMatrixsc& RigidBody3DState::Minv0()
{
  return m_Minv0;
}

const SparseMatrixsc& RigidBody3DState::Minv0() const
{
  return m_Minv0;
}

SparseMatrixsc& RigidBody3DState::Minv()
{
  return m_Minv;
}

const SparseMatrixsc& RigidBody3DState::Minv() const
{
  return m_Minv;
}

bool RigidBody3DState::isKinematicallyScripted( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_fixed.size() );
  return m_fixed[bdy_idx];
}

const std::vector<std::unique_ptr<RigidBodyGeometry>>& RigidBody3DState::geometry() const
{
  return m_geometry;
}

const std::vector<unsigned>& RigidBody3DState::indices() const
{
  return m_geometry_indices;
}

const RigidBodyGeometry& RigidBody3DState::getGeometryOfBody( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_geometry_indices.size() );
  return *m_geometry[ m_geometry_indices[ bdy_idx ] ];
}

unsigned RigidBody3DState::getGeometryIndexOfBody( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_geometry_indices.size() );
  return m_geometry_indices[ bdy_idx ];
}

const scalar& RigidBody3DState::getTotalMass( const unsigned body ) const
{
  assert( body < nbodies() );
  return M().valuePtr()[ 3 * body ];
}

const Eigen::Map<const Matrix33sr> RigidBody3DState::getInertia( const unsigned body ) const
{
  assert( body < nbodies() );
  assert( ( Eigen::Map<const Matrix33sr>( &M().valuePtr()[ 3 * nbodies() + 9 * body ] ) - Eigen::Map<const Matrix33sr>( &M().valuePtr()[ 3 * nbodies() + 9 * body ] ).transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-12 );
  return Eigen::Map<const Matrix33sr>( &M().valuePtr()[ 3 * nbodies() + 9 * body ] );
}

void RigidBody3DState::updateMandMinv()
{
  assert( unsigned( m_M0.nonZeros() ) == 6 * nbodies() );
  assert( m_M0.nonZeros() == m_Minv0.nonZeros() );
  assert( unsigned( m_M.nonZeros() ) == 12 * nbodies() );
  assert( m_M.nonZeros() == m_Minv.nonZeros() );

  for( unsigned bdy_idx = 0; bdy_idx < m_nbodies; ++bdy_idx )
  {
    // Orientation of the ith body
    const Eigen::Map<const Matrix33sr> R{ m_q.segment<9>( 3 * m_nbodies + 9 * bdy_idx ).data() };
    assert( fabs( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() ) <= 1.0e-9 );
    assert( fabs( R.determinant() - 1.0 ) <= 1.0e-9 );

    // Inertia tensor of the ith body
    {
      Eigen::Map<Matrix33sc> I{ &m_M.data().value( 3 * m_nbodies + 9 * bdy_idx ) };
      const Eigen::Map<const Vector3s> I0{ &m_M0.data().value( 3 * m_nbodies + 3 * bdy_idx ) };
      I = R * I0.asDiagonal() * R.transpose();
      assert( ( I - I.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-12 );
      assert( I.determinant() > 0.0 );
    }

    // Inverse of the inertia tensor of the ith body
    {
      Eigen::Map<Matrix33sc> Iinv{ &m_Minv.data().value( 3 * m_nbodies + 9 * bdy_idx ) };
      const Eigen::Map<const Vector3s> Iinv0{ &m_Minv0.data().value( 3 * m_nbodies + 3 * bdy_idx ) };
      Iinv = R * Iinv0.asDiagonal() * R.transpose();
      assert( ( Iinv - Iinv.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-12 );
      assert( Iinv.determinant() > 0.0 );
    }
  }

  assert( MathUtilities::isIdentity( m_M * m_Minv, 1.0e-9 ) );
}

std::vector<std::unique_ptr<Force>>& RigidBody3DState::forces()
{
  return m_forces;
}

const std::vector<std::unique_ptr<Force>>& RigidBody3DState::forces() const
{
  return m_forces;
}

void RigidBody3DState::addForce( const Force& new_force )
{
  m_forces.emplace_back( std::unique_ptr<Force>{ new_force.clone() } );
}

void RigidBody3DState::addStaticPlane( const StaticPlane& new_plane )
{
  m_static_planes.emplace_back( new_plane );
}

StaticPlane& RigidBody3DState::staticPlane( const std::vector<StaticPlane>::size_type plane_idx )
{
  assert( plane_idx < m_static_planes.size() );
  return m_static_planes[plane_idx];
}

const StaticPlane& RigidBody3DState::staticPlane( const std::vector<StaticPlane>::size_type plane_idx ) const
{
  assert( plane_idx < m_static_planes.size() );
  return m_static_planes[plane_idx];
}

const std::vector<StaticPlane>& RigidBody3DState::staticPlanes() const
{
  return m_static_planes;
}

std::vector<StaticPlane>::size_type RigidBody3DState::numStaticPlanes() const
{
  return m_static_planes.size();
}

void RigidBody3DState::deleteStaticPlane( const std::vector<StaticPlane>::size_type plane_index )
{
  assert( plane_index < m_static_planes.size() );
  m_static_planes.erase( m_static_planes.begin() + plane_index );
}

void RigidBody3DState::addStaticCylinder( const StaticCylinder& new_cylinder )
{
  m_static_cylinders.emplace_back( new_cylinder );
}

StaticCylinder& RigidBody3DState::staticCylinder( const std::vector<StaticCylinder>::size_type cylinder_index )
{
  assert( cylinder_index < m_static_cylinders.size() );
  return m_static_cylinders[cylinder_index];
}

const StaticCylinder& RigidBody3DState::staticCylinder( const std::vector<StaticCylinder>::size_type cylinder_index ) const
{
  assert( cylinder_index < m_static_cylinders.size() );
  return m_static_cylinders[cylinder_index];
}

const std::vector<StaticCylinder>& RigidBody3DState::staticCylinders() const
{
  return m_static_cylinders;
}

std::vector<StaticCylinder>::size_type RigidBody3DState::numStaticCylinders() const
{
  return m_static_cylinders.size();
}

void RigidBody3DState::addPlanarPortal( const PlanarPortal& planar_portal )
{
  m_planar_portals.emplace_back( planar_portal );
}

std::vector<PlanarPortal>::size_type RigidBody3DState::numPlanarPortals() const
{
  return m_planar_portals.size();
}

const PlanarPortal& RigidBody3DState::planarPortal( const std::vector<PlanarPortal>::size_type portal_index ) const
{
  assert( portal_index < m_planar_portals.size() );
  return m_planar_portals[portal_index];
}

void RigidBody3DState::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );

  Utilities::serialize( m_nbodies, output_stream );
  MathUtilities::serialize( m_q, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_M0, output_stream );
  MathUtilities::serialize( m_Minv0, output_stream );
  MathUtilities::serialize( m_M, output_stream );
  MathUtilities::serialize( m_Minv, output_stream );
  Utilities::serialize( m_fixed, output_stream );
  Utilities::serialize( m_geometry, output_stream );
  Utilities::serialize( m_geometry_indices, output_stream );
  Utilities::serialize( m_forces, output_stream );
  Utilities::serialize( m_static_planes, output_stream );
  Utilities::serialize( m_static_cylinders, output_stream );
  Utilities::serialize( m_planar_portals, output_stream );
}

static std::vector<std::unique_ptr<RigidBodyGeometry>> deserializeGeometry( std::istream& input_stream )
{
  const std::vector<std::unique_ptr<RigidBodyGeometry>>::size_type ngeo{ Utilities::deserialize<std::vector<std::unique_ptr<RigidBodyGeometry>>::size_type>( input_stream ) };
  std::vector<std::unique_ptr<RigidBodyGeometry>> geometry( ngeo );
  assert( geometry.size() == ngeo );
  for( std::vector<std::unique_ptr<RigidBodyGeometry>>::size_type geo_idx = 0; geo_idx < geometry.size(); ++geo_idx )
  {
    // Read in the geometry type
    const RigidBodyGeometryType geo_type{ Utilities::deserialize<RigidBodyGeometryType>( input_stream ) };
    switch( geo_type )
    {
      case RigidBodyGeometryType::BOX:
        geometry[geo_idx].reset( new RigidBodyBox{ input_stream } );
        break;
      case RigidBodyGeometryType::SPHERE:
        geometry[geo_idx].reset( new RigidBodySphere{ input_stream } );
        break;
      case RigidBodyGeometryType::STAPLE:
        std::cerr << "Staple geometry not yet supported in RigidBody3DState::deserialize" << std::endl;
        std::cerr << "Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      case RigidBodyGeometryType::TRIANGLE_MESH:
        geometry[geo_idx].reset( new RigidBodyTriangleMesh{ input_stream } );
        break;
    }
  }
  return geometry;
}

static std::vector<std::unique_ptr<Force>> deserializeForces( std::istream& input_stream )
{
  const std::vector<std::unique_ptr<Force>>::size_type nforces{ Utilities::deserialize<std::vector<std::unique_ptr<Force>>::size_type>( input_stream ) };
  std::vector<std::unique_ptr<Force>> forces( nforces );
  assert( forces.size() == nforces );
  for( std::vector<std::unique_ptr<Force>>::size_type force_idx = 0; force_idx < forces.size(); ++force_idx )
  {
    // Read in the force name
    const std::string force_name{ StringUtilities::deserialize( input_stream ) };
    if( "near_earth_gravity" == force_name )
    {
      forces[force_idx].reset( new NearEarthGravityForce{ input_stream } );
    }
    else
    {
      std::cerr << "Unknown force in deserialize." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }
  return forces;
}

void RigidBody3DState::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_nbodies = Utilities::deserialize<unsigned>( input_stream );
  m_q = MathUtilities::deserialize<VectorXs>( input_stream );
  m_v = MathUtilities::deserialize<VectorXs>( input_stream );
  MathUtilities::deserialize( m_M0, input_stream );
  MathUtilities::deserialize( m_Minv0, input_stream );
  MathUtilities::deserialize( m_M, input_stream );
  MathUtilities::deserialize( m_Minv, input_stream );
  m_fixed = Utilities::deserializeVector<bool>( input_stream );
  m_geometry = deserializeGeometry( input_stream );
  m_geometry_indices = Utilities::deserializeVector<unsigned>( input_stream );
  m_forces = deserializeForces( input_stream );
  m_static_planes = Utilities::deserializeVector<StaticPlane>( input_stream );
  m_static_cylinders = Utilities::deserializeVector<StaticCylinder>( input_stream );
  m_planar_portals = Utilities::deserializeVector<PlanarPortal>( input_stream );
}
