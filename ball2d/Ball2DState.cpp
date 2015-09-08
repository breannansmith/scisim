// Ball2DState.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "Ball2DState.h"

#include "SCISim/Math/MathUtilities.h"
#include "SCISim/Utilities.h"
#include "SCISim/StringUtilities.h"
#include "Forces/Ball2DGravityForce.h"

#include "StaticGeometry/StaticDrum.h"
#include "StaticGeometry/StaticPlane.h"
#include "Portals/PlanarPortal.h"

#include <iostream>

void swap( Ball2DState& first, Ball2DState& second )
{
  using std::swap;
  swap( first.m_q, second.m_q );
  swap( first.m_v, second.m_v );
  swap( first.m_r, second.m_r );
  swap( first.m_fixed, second.m_fixed );
  swap( first.m_M, second.m_M );
  swap( first.m_Minv, second.m_Minv );
  swap( first.m_static_drums, second.m_static_drums );
  swap( first.m_static_planes, second.m_static_planes );
  swap( first.m_planar_portals, second.m_planar_portals );
  swap( first.m_forces, second.m_forces );
}

Ball2DState::Ball2DState()
: m_q()
, m_v()
, m_r()
, m_fixed()
, m_M()
, m_Minv()
, m_static_drums()
, m_static_planes()
, m_planar_portals()
, m_forces()
{}

Ball2DState::Ball2DState( const Ball2DState& other )
: m_q( other.m_q )
, m_v( other.m_v )
, m_r( other.m_r )
, m_fixed( other.m_fixed )
, m_M( other.m_M )
, m_Minv( other.m_Minv )
, m_static_drums( other.m_static_drums )
, m_static_planes( other.m_static_planes )
, m_planar_portals( other.m_planar_portals )
, m_forces( Utilities::cloneVector( other.m_forces ) )
{}

static SparseMatrixsc createM( const VectorXs& m )
{
  SparseMatrixsc M{ static_cast<SparseMatrixsc::Index>( m.size() ), static_cast<SparseMatrixsc::Index>( m.size() ) };
  M.reserve( m.size() );
  for( int col = 0; col < m.size(); ++col )
  {
    M.startVec( col );
    const int row = col;
    M.insertBack( row, col ) = m( col );
  }
  M.finalize();
  return M;
}

static SparseMatrixsc createMinv( const VectorXs& m )
{
  SparseMatrixsc Minv{ static_cast<SparseMatrixsc::Index>( m.size() ), static_cast<SparseMatrixsc::Index>( m.size() ) };
  Minv.reserve( m.size() );
  for( int col = 0; col < m.size(); ++col )
  {
    Minv.startVec( col );
    const int row = col;
    Minv.insertBack( row, col ) = 1.0 / m( col );
  }
  Minv.finalize();
  return Minv;
}

Ball2DState::Ball2DState( const VectorXs& q, const VectorXs& v, const VectorXs& m, const VectorXs& r, const std::vector<bool>& fixed, const std::vector<StaticDrum>& drums, const std::vector<StaticPlane>& planes, const std::vector<PlanarPortal>& planar_portals, const std::vector<std::unique_ptr<Ball2DForce>>& forces )
: m_q( q )
, m_v( v )
, m_r( r )
, m_fixed( fixed )
, m_M( createM( m ) )
, m_Minv( createMinv( m ) )
, m_static_drums( drums )
, m_static_planes( planes )
, m_planar_portals( planar_portals )
, m_forces( Utilities::cloneVector( forces ) )
{
  assert( m_q.size() % 2 == 0 );
  assert( m_q.size() == m_v.size() );
  assert( m_q.size() == m_M.rows() );
  assert( m_q.size() == m_M.cols() );
  assert( m_q.size() == m_Minv.rows() );
  assert( m_q.size() == m_Minv.rows() );
  assert( m_q.size() / 2 == m_r.size() );
  assert( m_q.size() / 2 == int( m_fixed.size() ) );
  #ifndef NDEBUG
  const SparseMatrixsc should_be_id = m_M * m_Minv;
  Eigen::Map<const ArrayXs> should_be_id_data{ should_be_id.valuePtr(), should_be_id.nonZeros() };
  assert( ( ( should_be_id_data - 1.0 ).abs() <= 1.0e-6 ).all() );
  #endif
}

Ball2DState& Ball2DState::operator=( Ball2DState other )
{
  using std::swap;
  swap( *this, other );
  return *this;
}

Ball2DState::~Ball2DState()
{}

unsigned Ball2DState::nballs() const
{
  return m_fixed.size();
}

VectorXs& Ball2DState::q()
{
  return m_q;
}

VectorXs& Ball2DState::v()
{
  return m_v;
}

const VectorXs& Ball2DState::q() const
{
  return m_q;
}

const VectorXs& Ball2DState::v() const
{
  return m_v;
}

const VectorXs& Ball2DState::r() const
{
  return m_r;
}

const SparseMatrixsc& Ball2DState::M() const
{
  return m_M;
}

const SparseMatrixsc& Ball2DState::Minv() const
{
  return m_Minv;
}

std::vector<StaticPlane>& Ball2DState::staticPlanes()
{
  return m_static_planes;
}

const std::vector<StaticDrum>& Ball2DState::staticDrums() const
{
  return m_static_drums;
}

const std::vector<StaticPlane>& Ball2DState::staticPlanes() const
{
  return m_static_planes;
}

std::vector<PlanarPortal>& Ball2DState::planarPortals()
{
  return m_planar_portals;
}

const std::vector<PlanarPortal>& Ball2DState::planarPortals() const
{
  return m_planar_portals;
}

scalar Ball2DState::computeKineticEnergy() const
{
  return 0.5 * m_v.dot( m_M * m_v ) ;
}

scalar Ball2DState::computePotentialEnergy() const
{
  scalar U{ 0.0 };
  for( const std::unique_ptr<Ball2DForce>& force : m_forces )
  {
    U += force->computePotential( m_q, m_M, m_r );
  }
  return U;
}

scalar Ball2DState::computeTotalEnergy() const
{
  return computeKineticEnergy() + computePotentialEnergy();
}

Vector2s Ball2DState::computeMomentum() const
{
  Vector2s p{ Vector2s::Zero() };

  assert( m_v.size() % 2 == 0 );
  for( int i = 0; i < m_v.size(); i += 2 )
  {
    assert( m_M.valuePtr()[ i ] == m_M.valuePtr()[ i + 1 ] );
    p += m_M.valuePtr()[ i ] * m_v.segment<2>( i );
  }

  return p;
}

scalar Ball2DState::computeAngularMomentum() const
{
  scalar L{ 0 };

  assert( m_q.size() % 2 == 0 ); assert( m_q.size() == m_v.size() );
  for( int i = 0; i < m_v.size(); i += 2 )
  {
    assert( m_M.valuePtr()[ i ] == m_M.valuePtr()[ i + 1 ] );
    L += m_M.valuePtr()[ i ] * mathutils::cross( m_q.segment<2>( i ), m_v.segment<2>( i ) );
  }

  return L;
}

Vector4s Ball2DState::computeBoundingBox() const
{
  Vector4s box{ SCALAR_INFINITY, -SCALAR_INFINITY, SCALAR_INFINITY, -SCALAR_INFINITY };

  // Loop over all balls
  for( int i = 0; i < m_r.size(); ++i )
  {
    const scalar& r = m_r( i );
    const scalar& x = m_q( 2 * i );
    const scalar& y = m_q( 2 * i + 1 );
    box(0) = std::min( box(0), x - r );
    box(1) = std::max( box(1), x + r );
    box(2) = std::min( box(2), y - r );
    box(3) = std::max( box(3), y + r );
  }

  // Loop over all drums
  for( std::vector<StaticDrum>::size_type i = 0; i < m_static_drums.size(); ++i )
  {
    const scalar& r = m_static_drums[i].r();
    const scalar& x = m_static_drums[i].x().x();
    const scalar& y = m_static_drums[i].x().y();
    box(0) = std::min( box(0), x - r );
    box(1) = std::max( box(1), x + r );
    box(2) = std::min( box(2), y - r );
    box(3) = std::max( box(3), y + r );
  }

  // TODO: Halfplanes?
  // TODO: Planar portals?

  return box;
}

void Ball2DState::accumulateForce( const VectorXs& q, const VectorXs& v, VectorXs& force_accm ) const
{
  for( const std::unique_ptr<Ball2DForce>& force : m_forces )
  {
    force->computeForce( q, v, m_M, m_r, force_accm );
  }
}

void Ball2DState::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  mathutils::serialize( m_q, output_stream );
  mathutils::serialize( m_v, output_stream );
  mathutils::serialize( m_r, output_stream );
  Utilities::serializeVectorBuiltInType( m_fixed, output_stream );
  mathutils::serialize( m_M, output_stream );
  mathutils::serialize( m_Minv, output_stream );
  Utilities::serializeVectorCustomType( m_static_drums, output_stream );
  Utilities::serializeVectorCustomType( m_static_planes, output_stream );
  Utilities::serializeVectorCustomType( m_planar_portals, output_stream );
  Utilities::serializeVectorCustomTypePointers( m_forces, output_stream );
}

void Ball2DState::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );

  m_q = mathutils::deserialize<VectorXs>( input_stream );
  m_v = mathutils::deserialize<VectorXs>( input_stream );
  m_r = mathutils::deserialize<VectorXs>( input_stream );
  assert( ( m_r.array() > 0.0 ).all() );
  Utilities::deserializeVectorBuiltInType( m_fixed, input_stream ); assert( input_stream.good() );
  mathutils::deserialize( m_M, input_stream ); assert( input_stream.good() );
  // TODO: Assert data is all positive
  mathutils::deserialize( m_Minv, input_stream ); assert( input_stream.good() );
  // TODO: Assert data is all positive
  Utilities::deserializeVectorCustomType( m_static_drums, input_stream ); assert( input_stream.good() );
  Utilities::deserializeVectorCustomType( m_static_planes, input_stream ); assert( input_stream.good() );
  Utilities::deserializeVectorCustomType( m_planar_portals, input_stream ); assert( input_stream.good() );

  // TODO: Pull this into a utility function along with some code in Ball2DForce
  {
    std::vector<std::unique_ptr<Ball2DForce>>::size_type num_forces;
    input_stream.read( (char*) &num_forces, sizeof(std::vector<std::unique_ptr<Ball2DForce>>::size_type) );
    m_forces.resize( num_forces );
    for( std::vector<std::unique_ptr<Ball2DForce>>::size_type force_idx = 0; force_idx < m_forces.size(); ++force_idx )
    {
      // Read in the force name
      const std::string force_name = StringUtilities::deserializeString( input_stream );
      if( "ball2d_gravity_force" == force_name )
      {
        m_forces[force_idx] = std::unique_ptr<Ball2DForce>{ new Ball2DGravityForce{ input_stream } };
      }
      else
      {
        std::cerr << "Unknown force in deserialize." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
  }
}

void Ball2DState::pushBallBack( const Vector2s& q, const Vector2s& v, const scalar& r, const scalar& m, const bool fixed )
{
  const unsigned original_num_balls{ nballs() };
  const unsigned new_num_balls{ original_num_balls + 1 };

  // Update the positions
  m_q.conservativeResize( 2 * new_num_balls );
  m_q.segment<2>( 2 * original_num_balls ) = q;
  // Update the velocities
  m_v.conservativeResize( 2 * new_num_balls );
  m_v.segment<2>( 2 * original_num_balls ) = v;
  // Update the radii
  m_r.conservativeResize( new_num_balls );
  m_r( original_num_balls ) = r;
  // Update fixed balls
  m_fixed.push_back( fixed );
  // Update the mass matrix
  {
    SparseMatrixsc M( 2 * new_num_balls, 2 * new_num_balls );
    M.reserve( 2 * new_num_balls );
    // Copy the old masses
    for( unsigned col = 0; col < 2 * original_num_balls; ++col )
    {
      M.startVec( col );
      const unsigned row = col;
      M.insertBack( row, col ) = m_M.valuePtr()[ col ];
    }
    // Insert the new masses
    M.startVec( 2 * original_num_balls );
    M.insertBack( 2 * original_num_balls, 2 * original_num_balls ) = m;
    M.startVec( 2 * original_num_balls + 1 );
    M.insertBack( 2 * original_num_balls + 1, 2 * original_num_balls + 1 ) = m;
    M.finalize();
    m_M.swap( M );
  }
  // Update the inverse mass matrix
  {
    SparseMatrixsc Minv( 2 * new_num_balls, 2 * new_num_balls );
    Minv.reserve( 2 * new_num_balls );
    // Copy the old inverse masses
    for( unsigned col = 0; col < 2 * original_num_balls; ++col )
    {
      Minv.startVec( col );
      const unsigned row = col;
      Minv.insertBack( row, col ) = m_Minv.valuePtr()[ col ];
    }
    // Insert the new inverse masses
    Minv.startVec( 2 * original_num_balls );
    Minv.insertBack( 2 * original_num_balls, 2 * original_num_balls ) = 1.0 / m;
    Minv.startVec( 2 * original_num_balls + 1 );
    Minv.insertBack( 2 * original_num_balls + 1, 2 * original_num_balls + 1 ) = 1.0 / m;
    Minv.finalize();
    m_Minv.swap( Minv );
  }
}
