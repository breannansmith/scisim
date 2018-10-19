#include "Integrator.h"

#include "Ball2DSim.h"

#include "scisim/ConstrainedMaps/ConstrainedMapUtilities.h"
#include "scisim/Utilities.h"

#ifdef USE_HDF5
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactSolution.h"
#include "scisim/HDF5File.h"
#endif

#include "ball2d/Ball2DUtilities.h"

Integrator::Integrator()
: m_dt( 0, 1 )
, m_unconstrained_map( nullptr )
, m_impact_operator( nullptr )
, m_friction_solver( nullptr )
, m_impact_map( nullptr )
, m_impact_friction_map( nullptr )
, m_cor( 0.0 )
, m_mu( 0.0 )
, m_style( Style::None )
{}

template<typename T>
T nullAwareClone( const T& other )
{
  return other == nullptr ? T() : other->clone();
}

static Integrator::Style determineStyle( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, const std::unique_ptr<ImpactOperator>& impact_operator,
                                         const std::unique_ptr<FrictionSolver>& friction_solver, const std::unique_ptr<ImpactMap>& impact_map,
                                         const std::unique_ptr<ImpactFrictionMap>& impact_friction_map )
{
  if( unconstrained_map == nullptr && impact_operator == nullptr && impact_map == nullptr && friction_solver == nullptr && impact_friction_map == nullptr )
  {
    return Integrator::Style::None;
  }
  else if( unconstrained_map != nullptr && impact_operator == nullptr && impact_map == nullptr && friction_solver == nullptr && impact_friction_map == nullptr )
  {
    return Integrator::Style::Unconstrained;
  }
  else if( unconstrained_map != nullptr && impact_operator != nullptr && impact_map != nullptr && friction_solver == nullptr && impact_friction_map == nullptr )
  {
    return Integrator::Style::Impact;
  }
  else if( unconstrained_map != nullptr && impact_operator == nullptr && impact_map == nullptr && friction_solver != nullptr && impact_friction_map != nullptr )
  {
    return Integrator::Style::ImpactFriction;
  }
  else
  {
    // Should not be possible, but check for it.
    assert(false);
    return Integrator::Style::None;
  }
}

Integrator::Integrator( const Rational<std::intmax_t>& dt, const std::unique_ptr<UnconstrainedMap>& unconstrained_map,
                        const std::unique_ptr<ImpactOperator>& impact_operator, const std::unique_ptr<FrictionSolver>& friction_solver,
                        const std::unique_ptr<ImpactMap>& impact_map, const std::unique_ptr<ImpactFrictionMap>& impact_friction_map,
                        const scalar& cor, const scalar& mu )
: m_dt( dt )
, m_unconstrained_map( nullAwareClone(unconstrained_map) )
, m_impact_operator( nullAwareClone(impact_operator) )
, m_friction_solver( nullAwareClone(friction_solver) )
, m_impact_map( nullAwareClone(impact_map) )
, m_impact_friction_map( nullAwareClone(impact_friction_map) )
, m_cor( cor )
, m_mu( mu )
, m_style( determineStyle( unconstrained_map, impact_operator, friction_solver, impact_map, impact_friction_map ) )
{
  assert( m_dt.positive() );
}

Integrator::Integrator( const Integrator& other )
: m_dt( other.m_dt )
, m_unconstrained_map( nullAwareClone(other.m_unconstrained_map) )
, m_impact_operator( nullAwareClone(other.m_impact_operator) )
, m_friction_solver( nullAwareClone(other.m_friction_solver) )
, m_impact_map( nullAwareClone(other.m_impact_map) )
, m_impact_friction_map( nullAwareClone(other.m_impact_friction_map) )
, m_cor( other.m_cor )
, m_mu( other.m_mu )
, m_style( other.m_style )
{}

Integrator& Integrator::operator=( Integrator other )
{
  using std::swap;
  swap( m_dt, other.m_dt );
  swap( m_unconstrained_map, other.m_unconstrained_map );
  swap( m_impact_operator, other.m_impact_operator );
  swap( m_friction_solver, other.m_friction_solver );
  swap( m_impact_map, other.m_impact_map );
  swap( m_impact_friction_map, other.m_impact_friction_map );
  swap( m_cor, other.m_cor );
  swap( m_mu, other.m_mu );
  swap( m_style, other.m_style );
  return *this;
}

void Integrator::step( const int next_iter, PythonScripting& scripting, Ball2DSim& sim )
{
  switch( m_style )
  {
    case Style::None:
    {
      break;
    }
    case Style::Unconstrained:
    {
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map );
      break;
    }
    case Style::Impact:
    {
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_cor, *m_impact_map );
      break;
    }
    case Style::ImpactFriction:
    {
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map, m_cor, m_mu, *m_friction_solver, *m_impact_friction_map );
      break;
    }
  }
}

#ifdef USE_HDF5
void Integrator::stepWithForceOutput( const int next_iter, PythonScripting& scripting, Ball2DSim& sim, HDF5File& force_file )
{
  switch( m_style )
  {
    case Style::None:
    {
      break;
    }
    case Style::Unconstrained:
    {
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map );
      break;
    }
    case Style::Impact:
    {
      ImpactSolution impact_solution;
      if( force_file.is_open() )
      {
        m_impact_map->exportForcesNextStep( impact_solution );
      }
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_cor, *m_impact_map );
      if( force_file.is_open() )
      {
        impact_solution.writeSolution( force_file );
      }
      break;
    }
    case Style::ImpactFriction:
    {
      if( force_file.is_open() )
      {
        m_impact_friction_map->exportForcesNextStep( force_file );
      }
      sim.flow( scripting, next_iter, m_dt, *m_unconstrained_map, m_cor, m_mu, *m_friction_solver, *m_impact_friction_map );
      break;
    }
  }
}
#endif

void Integrator::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( m_dt, output_stream );
  Ball2DUtilities::serialize( m_unconstrained_map, output_stream );
  ConstrainedMapUtilities::serialize( m_impact_operator, output_stream );
  ConstrainedMapUtilities::serialize( m_friction_solver, output_stream );
  ConstrainedMapUtilities::serialize( m_impact_map, output_stream );
  ConstrainedMapUtilities::serialize( m_impact_friction_map, output_stream );
  Utilities::serialize( m_cor, output_stream );
  Utilities::serialize( m_mu, output_stream );
  Utilities::serialize( m_style, output_stream );
}

void Integrator::deserialize( std::istream& input_stream )
{
  m_dt = Utilities::deserialize<Rational<std::intmax_t>>( input_stream );
  assert( m_dt.positive() );
  m_unconstrained_map = Ball2DUtilities::deserializeUnconstrainedMap( input_stream );
  m_impact_operator = ConstrainedMapUtilities::deserializeImpactOperator( input_stream );
  m_friction_solver = ConstrainedMapUtilities::deserializeFrictionSolver( input_stream );
  m_impact_map = ConstrainedMapUtilities::deserializeImpactMap( input_stream );
  m_impact_friction_map = ConstrainedMapUtilities::deserializeImpactFrictionMap( input_stream );
  m_cor = Utilities::deserialize<scalar>( input_stream );
  assert( m_cor >= 0.0 ); assert( m_cor <= 1.0 );
  m_mu = Utilities::deserialize<scalar>( input_stream );
  assert( m_mu >= 0.0 );
  m_style = Utilities::deserialize<Style>( input_stream );
}

const Rational<std::intmax_t>& Integrator::dt() const
{
  return m_dt;
}

const Integrator::Style& Integrator::style() const
{
  return m_style;
}
