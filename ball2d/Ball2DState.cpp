#include "Ball2DState.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/StringUtilities.h"
#include "Forces/Ball2DGravityForce.h"

#include "StaticGeometry/StaticDrum.h"
#include "StaticGeometry/StaticPlane.h"
#include "Portals/PlanarPortal.h"

#include <iostream>

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
, m_forces( Utilities::clone( other.m_forces ) )
{}

Ball2DState& Ball2DState::operator=( const Ball2DState& other )
{
  Ball2DState copy{ other };
  using std::swap;
  swap( *this, copy );
  return *this;
}

static SparseMatrixsc createM( const VectorXs& m )
{
  SparseMatrixsc M{ m.size(), m.size() };
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
  SparseMatrixsc Minv{ m.size(), m.size() };
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

void Ball2DState::setMass( const VectorXs& m )
{
  m_M = createM( m );
  m_Minv = createMinv( m );
  #ifndef NDEBUG
  const SparseMatrixsc should_be_id{ m_M * m_Minv };
  const Eigen::Map<const ArrayXs> should_be_id_data{ should_be_id.valuePtr(), should_be_id.nonZeros() };
  assert( ( ( should_be_id_data - 1.0 ).abs() <= 1.0e-6 ).all() );
  #endif
}

unsigned Ball2DState::nballs() const
{
  return unsigned( m_fixed.size() );
}

VectorXs& Ball2DState::q()
{
  return m_q;
}

VectorXs& Ball2DState::v()
{
  return m_v;
}

VectorXs& Ball2DState::r()
{
  return m_r;
}

std::vector<bool>& Ball2DState::fixed()
{
  return m_fixed;
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

std::vector<StaticDrum>& Ball2DState::staticDrums()
{
  return m_static_drums;
}

std::vector<StaticPlane>& Ball2DState::staticPlanes()
{
  return m_static_planes;
}

std::vector<PlanarPortal>& Ball2DState::planarPortals()
{
  return m_planar_portals;
}

const std::vector<StaticDrum>& Ball2DState::staticDrums() const
{
  return m_static_drums;
}

const std::vector<StaticPlane>& Ball2DState::staticPlanes() const
{
  return m_static_planes;
}

const std::vector<PlanarPortal>& Ball2DState::planarPortals() const
{
  return m_planar_portals;
}

std::vector<PlanarPortal>::size_type Ball2DState::numPlanarPortals() const
{
  return m_planar_portals.size();
}

std::vector<std::unique_ptr<Ball2DForce>>& Ball2DState::forces()
{
  return m_forces;
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
    L += m_M.valuePtr()[ i ] * MathUtilities::cross( m_q.segment<2>( i ), m_v.segment<2>( i ) );
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
  MathUtilities::serialize( m_q, output_stream );
  MathUtilities::serialize( m_v, output_stream );
  MathUtilities::serialize( m_r, output_stream );
  Utilities::serialize( m_fixed, output_stream );
  MathUtilities::serialize( m_M, output_stream );
  MathUtilities::serialize( m_Minv, output_stream );
  Utilities::serialize( m_static_drums, output_stream );
  Utilities::serialize( m_static_planes, output_stream );
  Utilities::serialize( m_planar_portals, output_stream );
  Utilities::serialize( m_forces, output_stream );
}

void Ball2DState::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );

  m_q = MathUtilities::deserialize<VectorXs>( input_stream );
  m_v = MathUtilities::deserialize<VectorXs>( input_stream );
  m_r = MathUtilities::deserialize<VectorXs>( input_stream );
  assert( ( m_r.array() > 0.0 ).all() );
  m_fixed = Utilities::deserialize<std::vector<bool>>( input_stream );
  MathUtilities::deserialize( m_M, input_stream );
  assert( m_M.rows() == m_M.cols() );
  assert( m_M.rows() == 2 * long(m_fixed.size()) );
  // TODO: Assert data is all positive
  MathUtilities::deserialize( m_Minv, input_stream );
  // TODO: Assert data is all positive
  m_static_drums = Utilities::deserialize<std::vector<StaticDrum>>( input_stream );
  m_static_planes = Utilities::deserialize<std::vector<StaticPlane>>( input_stream );
  m_planar_portals = Utilities::deserialize<std::vector<PlanarPortal>>( input_stream );

  // TODO: Pull this into a utility function along with some code in Ball2DForce
  {
    std::vector<std::unique_ptr<Ball2DForce>>::size_type num_forces;
    input_stream.read( (char*) &num_forces, sizeof(std::vector<std::unique_ptr<Ball2DForce>>::size_type) );
    m_forces.resize( num_forces );
    for( std::vector<std::unique_ptr<Ball2DForce>>::size_type force_idx = 0; force_idx < m_forces.size(); ++force_idx )
    {
      // Read in the force name
      const std::string force_name{ StringUtilities::deserialize( input_stream ) };
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

bool Ball2DState::empty() const
{
  return m_q.size() == 0;
}
