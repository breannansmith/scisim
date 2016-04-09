// RigidBody2DState.cpp
//
// Breannan Smith
// Last updated: 01/05/2016

#include "RigidBody2DState.h"

#include "scisim/Utilities.h"
#include "scisim/Math/MathUtilities.h"
#include "scisim/StringUtilities.h"

#include "BoxGeometry.h"
#include "CircleGeometry.h"
#include "NearEarthGravityForce.h"

static SparseMatrixsc generateM( const VectorXs& m )
{
  SparseMatrixsc M{ SparseMatrixsc::Index( m.size() ), SparseMatrixsc::Index( m.size() ) };
  M.reserve( SparseMatrixsc::Index( m.size() ) );
  for( int col = 0; col < m.size(); ++col )
  {
    M.startVec( col );
    const int row{ col };
    M.insertBack( row, col ) = m( col );
  }
  M.finalize();
  M.makeCompressed();
  return M;
}

static SparseMatrixsc generateMinv( const VectorXs& m )
{
  SparseMatrixsc Minv{ SparseMatrixsc::Index( m.size() ), SparseMatrixsc::Index( m.size() ) };
  Minv.reserve( SparseMatrixsc::Index( m.size() ) );
  for( int col = 0; col < m.size(); ++col )
  {
    Minv.startVec( col );
    const int row{ col };
    Minv.insertBack( row, col ) = 1.0 / m( col );
  }
  Minv.finalize();
  Minv.makeCompressed();
  return Minv;
}

#ifndef NDEBUG
void RigidBody2DState::checkStateConsistency()
{
  assert( m_q.size() % 3 == 0 );
  assert( m_q.size() == m_v.size() );

  assert( m_M.rows() == m_q.size() );
  assert( m_M.cols() == m_q.size() );
  assert( ( Eigen::Map<const ArrayXs>{ m_M.valuePtr(), m_M.nonZeros() } > 0.0 ).all() );
  for( unsigned bdy_idx = 0; bdy_idx < m_q.size() / 3; ++bdy_idx )
  {
    assert( m_M.valuePtr()[ 3 * bdy_idx ] == m_M.valuePtr()[ 3 * bdy_idx + 1 ] );
  }

  assert( m_Minv.rows() == m_q.size() );
  assert( m_Minv.cols() == m_q.size() );
  assert( ( Eigen::Map<const ArrayXs>{ m_Minv.valuePtr(), m_Minv.nonZeros() } > 0.0 ).all() );
  for( unsigned bdy_idx = 0; bdy_idx < m_q.size() / 3; ++bdy_idx )
  {
    assert( m_Minv.valuePtr()[ 3 * bdy_idx ] == m_Minv.valuePtr()[ 3 * bdy_idx + 1 ] );
  }

  // Check that the product of M and M^{-1} is the identity
  {
    const SparseMatrixsc prod{ m_M * m_Minv };
    assert( prod.nonZeros() == m_M.rows() );
    assert( ( ( Eigen::Map<const ArrayXs>{ prod.valuePtr(), prod.nonZeros() } - 1.0 ).abs() <= 1.0e-6 ).all() );
  }

  assert( static_cast<int>( m_geometry_indices.size() ) == m_q.size() / 3 );
  assert( ( m_geometry_indices.array() < unsigned( m_geometry.size() ) ).all() );

  assert( static_cast<int>( m_fixed.size() ) == m_q.size() / 3 );
}
#endif

RigidBody2DState::RigidBody2DState( const VectorXs& q, const VectorXs& v, const VectorXs& m, const std::vector<bool>& fixed, const VectorXu& geometry_indices, const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, const std::vector<std::unique_ptr<RigidBody2DForce>>& forces, const std::vector<RigidBody2DStaticPlane>& planes, const std::vector<PlanarPortal>& planar_portals )
: m_q( q )
, m_v( v )
, m_M( generateM( m ) )
, m_Minv( generateMinv( m ) )
, m_fixed( fixed )
, m_geometry_indices( geometry_indices )
, m_geometry( Utilities::cloneVector( geometry ) )
, m_forces( Utilities::cloneVector( forces ) )
, m_planes( planes )
, m_planar_portals( planar_portals )
{
  #ifndef NDEBUG
  checkStateConsistency();
  #endif
}

RigidBody2DState::RigidBody2DState( const RigidBody2DState& rhs )
: m_q( rhs.m_q )
, m_v( rhs.m_v )
, m_M( rhs.m_M )
, m_Minv( rhs.m_Minv )
, m_fixed( rhs.m_fixed )
, m_geometry_indices( rhs.m_geometry_indices )
, m_geometry( Utilities::cloneVector( rhs.m_geometry ) )
, m_forces( Utilities::cloneVector( rhs.m_forces ) )
, m_planes( rhs.m_planes )
, m_planar_portals( rhs.m_planar_portals )
{
  #ifndef NDEBUG
  checkStateConsistency();
  #endif
}

RigidBody2DState& RigidBody2DState::operator=( const RigidBody2DState& rhs )
{
  RigidBody2DState copy{ rhs };
  using std::swap;
  swap( *this, copy );
  return *this;
}

VectorXs& RigidBody2DState::q()
{
  return m_q;
}

const VectorXs& RigidBody2DState::q() const
{
  return m_q;
}

VectorXs& RigidBody2DState::v()
{
  return m_v;
}

const VectorXs& RigidBody2DState::v() const
{
  return m_v;
}

const SparseMatrixsc& RigidBody2DState::M() const
{
  return m_M;
}

const SparseMatrixsc& RigidBody2DState::Minv() const
{
  return m_Minv;
}

const scalar& RigidBody2DState::m( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_q.size() / 3 );
  assert( m_M.valuePtr()[ 3 * bdy_idx ] == m_M.valuePtr()[ 3 * bdy_idx + 1 ] );
  assert( m_M.valuePtr()[ 3 * bdy_idx ] > 0.0 );
  return m_M.valuePtr()[ 3 * bdy_idx ];
}

const scalar& RigidBody2DState::I( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_q.size() / 3 );
  assert( m_M.valuePtr()[ 3 * bdy_idx + 2 ] > 0.0 );
  return m_M.valuePtr()[ 3 * bdy_idx + 2 ];
}

bool RigidBody2DState::fixed( const int idx ) const
{
  assert( idx >= 0 );
  assert( idx < static_cast<int>( m_fixed.size() ) );
  return m_fixed[idx];
}

void RigidBody2DState::addBody( const Vector2s& x, const scalar& theta, const Vector2s& v, const scalar& omega, const scalar& rho, const unsigned geo_idx, const bool fixed )
{
  assert( rho > 0.0 );
  assert( geo_idx < m_geometry.size() );

  assert( m_q.size() % 3 == 0 );
  const unsigned original_num_bodies{ unsigned( m_q.size() / 3 ) };
  const unsigned new_num_bodies{ original_num_bodies + 1 };

  // Format: x0, y0, theta0, x1, y1, theta1, ...
  m_q.conservativeResize( 3 * new_num_bodies );
  m_q.segment<3>( 3 * original_num_bodies ) << x.x(), x.y(), theta;

  // Format: vx0, vy0, omega0, vx1, vy1, omega1, ...
  m_v.conservativeResize( 3 * new_num_bodies );
  m_v.segment<3>( 3 * original_num_bodies ) << v.x(), v.y(), omega;

  // Update the geometry references
  m_geometry_indices.conservativeResize( new_num_bodies );
  m_geometry_indices( original_num_bodies ) = geo_idx;

  // Update fixed body tags
  m_fixed.push_back( fixed );

  scalar m;
  scalar I;
  m_geometry[geo_idx]->computeMassAndInertia( rho, m, I );

  // Update the mass matrix
  {
    SparseMatrixsc M{ SparseMatrixsc::Index( 3 * new_num_bodies ), SparseMatrixsc::Index( 3 * new_num_bodies ) };
    M.reserve( 3 * new_num_bodies );
    // Copy the old masses
    for( unsigned col = 0; col < 3 * original_num_bodies; ++col )
    {
      M.startVec( col );
      const unsigned row = col;
      M.insertBack( row, col ) = m_M.valuePtr()[ col ];
    }
    // Insert the new masses
    M.startVec( 3 * original_num_bodies );
    M.insertBack( 3 * original_num_bodies, 3 * original_num_bodies ) = m;
    M.startVec( 3 * original_num_bodies + 1 );
    M.insertBack( 3 * original_num_bodies + 1, 3 * original_num_bodies + 1 ) = m;
    M.startVec( 3 * original_num_bodies + 2 );
    M.insertBack( 3 * original_num_bodies + 2, 3 * original_num_bodies + 2 ) = I;
    M.finalize();
    m_M.swap( M );
  }
  // Update the inverse mass matrix
  {
    SparseMatrixsc Minv{ SparseMatrixsc::Index( 3 * new_num_bodies ), SparseMatrixsc::Index( 3 * new_num_bodies ) };
    Minv.reserve( 3 * new_num_bodies );
    // Copy the old inverse masses
    for( unsigned col = 0; col < 3 * original_num_bodies; ++col )
    {
      Minv.startVec( col );
      const unsigned row = col;
      Minv.insertBack( row, col ) = m_Minv.valuePtr()[ col ];
    }
    // Insert the new inverse masses
    Minv.startVec( 3 * original_num_bodies );
    Minv.insertBack( 3 * original_num_bodies, 3 * original_num_bodies ) = 1.0 / m;
    Minv.startVec( 3 * original_num_bodies + 1 );
    Minv.insertBack( 3 * original_num_bodies + 1, 3 * original_num_bodies + 1 ) = 1.0 / m;
    Minv.startVec( 3 * original_num_bodies + 2 );
    Minv.insertBack( 3 * original_num_bodies + 2, 3 * original_num_bodies + 2 ) = 1.0 / I;
    Minv.finalize();
    m_Minv.swap( Minv );
  }

  #ifndef NDEBUG
  checkStateConsistency();
  #endif
}

void RigidBody2DState::removeBodies( const Eigen::Ref<const VectorXu>& indices )
{
  if( indices.size() == 0 )
  {
    return;
  }

  // Use q to mark dofs for deletion
  for( unsigned delete_idx = 0; delete_idx < indices.size(); ++delete_idx )
  {
    m_q( 3 * indices[delete_idx] ) = SCALAR_NAN;
  }

  const unsigned nbodies_initial{ static_cast<unsigned>( m_q.size() ) / 3 };

  Eigen::Map<VectorXs> M_flat{ m_M.valuePtr(), m_M.nonZeros() };
  Eigen::Map<VectorXs> Minv_flat{ m_Minv.valuePtr(), m_Minv.nonZeros() };

  unsigned copy_to = 0;
  unsigned copy_from = 0;
  for( ; copy_from < nbodies_initial; copy_from++, copy_to++ )
  {
    while( copy_from < nbodies_initial && std::isnan( m_q( 3 * copy_from ) ) )
    {
      copy_from++;
    }
    if( copy_from == nbodies_initial )
    {
      break;
    }
    m_q.segment<3>( 3 * copy_to ) = m_q.segment<3>( 3 * copy_from );
    m_v.segment<3>( 3 * copy_to ) = m_v.segment<3>( 3 * copy_from );
    M_flat.segment<3>( 3 * copy_to ) = M_flat.segment<3>( 3 * copy_from );
    Minv_flat.segment<3>( 3 * copy_to ) = Minv_flat.segment<3>( 3 * copy_from );
    m_fixed[ copy_to ] = m_fixed[ copy_from ];
    m_geometry_indices( copy_to ) = m_geometry_indices( copy_from );
  }

  const unsigned new_num_dofs{ 3 * copy_to };

  m_q.conservativeResize( new_num_dofs );
  m_v.conservativeResize( new_num_dofs );
  //m_M.conservativeResize( new_num_dofs, new_num_dofs );
  //m_M.makeCompressed();
  //m_Minv.conservativeResize( new_num_dofs, new_num_dofs );
  //m_Minv.makeCompressed();
  m_fixed.resize( copy_to );
  m_fixed.shrink_to_fit();
  m_geometry_indices.conservativeResize( copy_to );

  // Note: Conservative resize on sparse matrix seems to cause issues...
  // Update the mass matrix
  {
    SparseMatrixsc M{ SparseMatrixsc::Index( new_num_dofs ), SparseMatrixsc::Index( new_num_dofs ) };
    M.reserve( new_num_dofs );
    // Copy the updated masses over
    for( unsigned col = 0; col < new_num_dofs; ++col )
    {
      M.startVec( col );
      M.insertBack( col, col ) = M_flat( col );
    }
    M.finalize();
    M.makeCompressed();
    m_M.swap( M );
  }

  // Update the inverse mass matrix
  {
    SparseMatrixsc Minv{ SparseMatrixsc::Index( new_num_dofs ), SparseMatrixsc::Index( new_num_dofs ) };
    Minv.reserve( new_num_dofs );
    // Copy the old inverse masses
    for( unsigned col = 0; col < new_num_dofs; ++col )
    {
      Minv.startVec( col );
      Minv.insertBack( col, col ) = Minv_flat( col );
    }
    Minv.finalize();
    Minv.makeCompressed();
    m_Minv.swap( Minv );
  }

  #ifndef NDEBUG
  checkStateConsistency();
  #endif
}

void RigidBody2DState::removeGeometry( const Eigen::Ref<const VectorXu>& indices )
{
  if( indices.size() == 0 )
  {
    return;
  }

  // Mark geometry slated for deletion
  for( unsigned delete_idx = 0; delete_idx < indices.size(); ++delete_idx )
  {
    m_geometry[indices[delete_idx]] = nullptr;
  }

  const unsigned ngeo_initial{ static_cast<unsigned>( m_geometry.size() ) };

  // Map from old geo indices to new geo indices. count, ngeo
  VectorXu index_map( ngeo_initial );

  unsigned copy_to = 0;
  unsigned copy_from = 0;
  for( ; copy_from < ngeo_initial; copy_from++, copy_to++ )
  {
    while( copy_from < ngeo_initial && m_geometry[copy_from] == nullptr )
    {
      #ifndef NDEBUG
      index_map(copy_from) = std::numeric_limits<unsigned>::max();
      #endif
      copy_from++;
    }
    if( copy_from == ngeo_initial )
    {
      break;
    }
    m_geometry[copy_to] = std::move(m_geometry[copy_from]);
    index_map[copy_from] = copy_to;
  }

  m_geometry.resize( copy_to );
  m_geometry.shrink_to_fit();

  for( unsigned bdy_idx = 0; bdy_idx < m_geometry_indices.size(); ++bdy_idx )
  {
    m_geometry_indices(bdy_idx) = index_map(m_geometry_indices(bdy_idx));
  }

  #ifndef NDEBUG
  checkStateConsistency();
  #endif
}

void RigidBody2DState::addCircleGeometry( const scalar& r )
{
  m_geometry.emplace_back( new CircleGeometry{ r } );
}

std::vector<std::unique_ptr<RigidBody2DGeometry>>& RigidBody2DState::geometry()
{
  return m_geometry;
}

const std::vector<std::unique_ptr<RigidBody2DGeometry>>& RigidBody2DState::geometry() const
{
  return m_geometry;
}

VectorXu& RigidBody2DState::geometryIndices()
{
  return m_geometry_indices;
}

const VectorXu& RigidBody2DState::geometryIndices() const
{
  return m_geometry_indices;
}

const std::unique_ptr<RigidBody2DGeometry>& RigidBody2DState::bodyGeometry( const unsigned bdy_idx ) const
{
  assert( bdy_idx < m_geometry_indices.size() ); assert( m_geometry_indices( bdy_idx ) < m_geometry.size() );
  return m_geometry[ m_geometry_indices( bdy_idx ) ];
}

const std::vector<std::unique_ptr<RigidBody2DForce>>& RigidBody2DState::forces() const
{
  return m_forces;
}

std::vector<RigidBody2DStaticPlane>& RigidBody2DState::planes()
{
  return m_planes;
}

const std::vector<RigidBody2DStaticPlane>& RigidBody2DState::planes() const
{
  return m_planes;
}

std::vector<PlanarPortal>& RigidBody2DState::planarPortals()
{
  return m_planar_portals;
}

const std::vector<PlanarPortal>& RigidBody2DState::planarPortals() const
{
  return m_planar_portals;
}

Array4s RigidBody2DState::computeBoundingBox() const
{
  const unsigned nbodies{ static_cast<unsigned>( m_q.size() / 3 ) };
  assert( nbodies > 0 );

  // For each body
  Array4s bounds;
  bounds.segment<2>( 0 ).setConstant( SCALAR_INFINITY );
  bounds.segment<2>( 2 ).setConstant( - SCALAR_INFINITY );
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    const Vector2s x{ m_q.segment<2>( 3 * bdy_idx ) };
    const scalar theta{ m_q( 3 * bdy_idx + 2) };
    Array2s body_min;
    Array2s body_max;
    m_geometry[ m_geometry_indices( bdy_idx ) ]->computeAABB( x, theta, body_min, body_max );
    bounds.segment<2>( 0 ) = bounds.segment<2>( 0 ).min( body_min );
    bounds.segment<2>( 2 ) = bounds.segment<2>( 2 ).max( body_max );
  }
  assert( ( bounds.segment<2>( 0 ) < bounds.segment<2>( 2 ) ).all() );

  return bounds;
}

void RigidBody2DState::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_q, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_M, output_stream );
  MathUtilities::serialize( m_Minv, output_stream );
  Utilities::serializeVectorBuiltInType( m_fixed, output_stream );
  MathUtilities::serialize( m_geometry_indices, output_stream );
  Utilities::serializeVectorCustomTypePointers( m_geometry, output_stream );
  Utilities::serializeVectorCustomTypePointers( m_forces, output_stream );
  Utilities::serializeVectorCustomType( m_planes, output_stream );
  Utilities::serializeVectorCustomType( m_planar_portals, output_stream );
}

static void deserializeGeo( std::istream& input_stream, std::vector<std::unique_ptr<RigidBody2DGeometry>>& geo )
{
  geo.clear();
  const std::vector<std::unique_ptr<RigidBody2DGeometry>>::size_type ngeo{ Utilities::deserialize<std::vector<std::unique_ptr<RigidBody2DGeometry>>::size_type>( input_stream ) };
  geo.resize( ngeo );
  for( std::vector<std::unique_ptr<RigidBody2DGeometry>>::size_type geo_idx = 0; geo_idx < ngeo; ++geo_idx )
  {
    // Read in the geometry type
    const RigidBody2DGeometryType geo_type{ Utilities::deserialize<RigidBody2DGeometryType>( input_stream ) };
    switch( geo_type )
    {
      case RigidBody2DGeometryType::CIRCLE:
      {
        geo[geo_idx].reset( new CircleGeometry{ input_stream } );
        break;
      }
      case RigidBody2DGeometryType::BOX:
      {
        geo[geo_idx].reset( new BoxGeometry{ input_stream } );
        break;
      }
    }
  }
}

static void deserializeForces( std::istream& input_stream, std::vector<std::unique_ptr<RigidBody2DForce>>& forces )
{
  using st = std::vector<std::unique_ptr<RigidBody2DForce>>::size_type;
  const st nforces{ Utilities::deserialize<st>( input_stream ) };
  forces.resize( nforces );
  for( st force_idx = 0; force_idx < nforces; ++force_idx )
  {
    const RigidBody2DForceType force_type{ Utilities::deserialize<RigidBody2DForceType>( input_stream ) };
    switch( force_type )
    {
      case RigidBody2DForceType::NEAR_EARTH_GRAVITY:
        forces[force_idx].reset( new NearEarthGravityForce{ input_stream } );
        break;
    }
  }
}

void RigidBody2DState::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );

  m_q = MathUtilities::deserialize<VectorXs>( input_stream );
  m_v = MathUtilities::deserialize<VectorXs>( input_stream );
  MathUtilities::deserialize( m_M, input_stream );
  MathUtilities::deserialize( m_Minv, input_stream );
  Utilities::deserializeVectorBuiltInType( m_fixed, input_stream );
  m_geometry_indices = MathUtilities::deserialize<VectorXu>( input_stream );
  deserializeGeo( input_stream, m_geometry );
  deserializeForces( input_stream, m_forces );
  Utilities::deserializeVectorCustomType( m_planes, input_stream );
  Utilities::deserializeVectorCustomType( m_planar_portals, input_stream );
}
