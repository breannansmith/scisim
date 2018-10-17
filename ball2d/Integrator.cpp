#include "Integrator.h"

#include "Ball2DSim.h"

// #include "rigidbody3d/PythonScripting.h"
// #include "rigidbody3d/RigidBody3DSim.h"
// #include "rigidbody3d/RigidBody3DUtilities.h"

// #include "scisim/ConstrainedMaps/ConstrainedMapUtilities.h"

// #include <iostream>
// #include "scisim/ConstrainedMaps/NullFrictionSolver.h"
// #include "scisim/UnconstrainedMaps/NullUnconstrainedMap.h"

// Integrator::DiscreteIntegrator()
// : m_iteration( 0 )
// , m_dt( 0, 1 )
// , m_unconstrained_map( nullptr )
// , m_impact_operator( nullptr )
// , m_friction_solver( nullptr )
// , m_impact_friction_map( nullptr )
// , m_cor( 0.0 )
// , m_mu( 0.0 )
// , m_reduce_bandwidth( false )
// , m_python_scripting()
// {}

template<typename T>
T nullAwareClone( const T& other )
{
  return other == nullptr ? T() : other->clone();
}

Integrator::Integrator( const Rational<std::intmax_t>& dt, const std::unique_ptr<UnconstrainedMap>& unconstrained_map, const std::unique_ptr<ImpactOperator>& impact_operator,
                        const std::unique_ptr<FrictionSolver>& friction_solver, const std::unique_ptr<ImpactMap>& impact_map, const std::unique_ptr<ImpactFrictionMap>& impact_friction_map,
                        const scalar& cor, const scalar& mu )
: m_dt( dt )
, m_unconstrained_map( nullAwareClone(unconstrained_map) )
, m_impact_operator( nullAwareClone(impact_operator) )
, m_friction_solver( nullAwareClone(friction_solver) )
, m_impact_map( nullAwareClone(impact_map) )
, m_impact_friction_map( nullAwareClone(impact_friction_map) )
, m_cor( cor )
, m_mu( mu )
, m_scripting()
{
  assert( m_dt.positive() );
  // TODO: Assert proper combination of maps here
}

// DiscreteIntegrator::DiscreteIntegrator( const DiscreteIntegrator& other )
// : m_iteration( 0 )
// , m_dt( other.m_dt )
// , m_unconstrained_map( nullAwareClone(other.m_unconstrained_map) )
// , m_impact_operator( nullAwareClone(other.m_impact_operator) )
// , m_friction_solver( nullAwareClone(other.m_friction_solver) )
// , m_impact_friction_map( nullAwareClone(other.m_impact_friction_map) )
// , m_cor( other.m_cor )
// , m_mu( other.m_mu )
// , m_reduce_bandwidth( other.m_reduce_bandwidth )
// , m_python_scripting()
// {
//   PythonScripting new_scripting{ other.m_python_scripting.path(), other.m_python_scripting.name() };
//   using std::swap;
//   swap( m_python_scripting, new_scripting );
// }

// DiscreteIntegrator& DiscreteIntegrator::operator=( DiscreteIntegrator other )
// {
//   using std::swap;

//   swap( m_iteration, other.m_iteration );
//   swap( m_dt, other.m_dt );
//   swap( m_unconstrained_map, other.m_unconstrained_map );
//   swap( m_impact_operator, other.m_impact_operator );
//   swap( m_friction_solver, other.m_friction_solver );
//   swap( m_impact_friction_map, other.m_impact_friction_map );
//   swap( m_cor, other.m_cor );
//   swap( m_mu, other.m_mu );
//   swap( m_reduce_bandwidth, other.m_reduce_bandwidth );
//   swap( m_python_scripting, other.m_python_scripting );

//   return *this;
// }

// void DiscreteIntegrator::setPythonCallback( const std::string& path, const std::string& module_name )
// {
//   PythonScripting new_scripting{ path, module_name };
//   using std::swap;
//   swap( m_python_scripting, new_scripting );
// }

// void DiscreteIntegrator::pythonStartOfSim( RigidBody3DSim& sim )
// {
//   m_python_scripting.setState( sim.getState() );
//   m_python_scripting.setInitialIterate( m_iteration );
//   m_python_scripting.startOfSimCallback();
//   m_python_scripting.forgetState();
// }

// void DiscreteIntegrator::pythonEndOfSim( RigidBody3DSim& sim )
// {
//   m_python_scripting.setState( sim.getState() );
//   m_python_scripting.endOfSimCallback();
//   m_python_scripting.forgetState();
// }

// const Rational<std::intmax_t>& DiscreteIntegrator::timestep() const
// {
//   return m_dt;
// }

void Integrator::step( const int next_iter, Ball2DSim& sim )
{
  // TODO: Precompute which thing we will call
  if( m_unconstrained_map == nullptr && m_impact_operator == nullptr && m_impact_map == nullptr && m_friction_solver == nullptr && m_impact_friction_map == nullptr )
  {
    return;
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_impact_map == nullptr && m_friction_solver == nullptr && m_impact_friction_map == nullptr )
  {
    sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator != nullptr && m_impact_map != nullptr && m_friction_solver == nullptr && m_impact_friction_map == nullptr )
  {
    sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_cor, *m_impact_map );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_impact_map == nullptr && m_friction_solver != nullptr && m_impact_friction_map != nullptr )
  {
    sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, m_cor, m_mu, *m_friction_solver, *m_impact_friction_map );
  }
}

// scalar DiscreteIntegrator::computeTime() const
// {
//   return scalar(std::intmax_t(m_iteration) * m_dt);
// }

// bool DiscreteIntegrator::frictionIsEnabled() const
// {
//   return m_impact_friction_map != nullptr;
// }

// //ImpactFrictionMap& DiscreteIntegrator::impactFrictionMap()
// //{
// //  assert(m_impact_friction_map != nullptr);
// //  return *m_impact_friction_map;
// //}

// std::unique_ptr<ImpactFrictionMap>& DiscreteIntegrator::impactFrictionMap()
// {
//   return m_impact_friction_map;
// }

// ImpactFrictionMap* DiscreteIntegrator::impactFrictionMapPointer()
// {
//   return m_impact_friction_map.get();
// }

// void DiscreteIntegrator::serialize( std::ostream& output_stream ) const
// {
//   Utilities::serializeBuiltInType( m_iteration, output_stream );
//   RationalTools::serialize( m_dt, output_stream );
//   RigidBody3DUtilities::serialize( m_unconstrained_map, output_stream );
//   ConstrainedMapUtilities::serialize( m_impact_operator, output_stream );
//   ConstrainedMapUtilities::serialize( m_friction_solver, output_stream );
//   ConstrainedMapUtilities::serialize( m_impact_friction_map, output_stream );
//   Utilities::serializeBuiltInType( m_cor, output_stream );
//   Utilities::serializeBuiltInType( m_mu, output_stream );
//   Utilities::serializeBuiltInType( m_reduce_bandwidth, output_stream );
//   m_python_scripting.serialize( output_stream );
// }

// void DiscreteIntegrator::deserialize( std::istream& input_stream )
// {
//   m_iteration = Utilities::deserialize<unsigned>( input_stream );
//   RationalTools::deserialize( m_dt, input_stream );
//   m_unconstrained_map = RigidBody3DUtilities::deserializeUnconstrainedMap( input_stream );
//   m_impact_operator = ConstrainedMapUtilities::deserializeImpactOperator( input_stream );
//   m_friction_solver = ConstrainedMapUtilities::deserializeFrictionSolver( input_stream );
//   m_impact_friction_map = ConstrainedMapUtilities::deserializeImpactFrictionMap( input_stream, "" );
//   m_cor = Utilities::deserialize<scalar>( input_stream );
//   m_mu = Utilities::deserialize<scalar>( input_stream );
//   m_reduce_bandwidth = Utilities::deserialize<bool>( input_stream );
//   {
//     PythonScripting new_scripting{ input_stream };
//     swap( m_python_scripting, new_scripting );
//   }
// }
