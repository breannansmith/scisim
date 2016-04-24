// ConstrainedMapUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "ConstrainedMapUtilities.h"

#include "scisim/StringUtilities.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GRROperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/GaussSeidelOperator.h"
#include "scisim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/ConstrainedMaps/Sobogus.h"
#include "scisim/HDF5File.cpp"

#ifdef IPOPT_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#include "scisim/ConstrainedMaps/FrictionMaps/SmoothMDPOperatorIpopt.h"
#endif

#ifdef QL_FOUND
#include "scisim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "scisim/ConstrainedMaps/FrictionMaps/LinearMDPOperatorQL.h"
#include "scisim/ConstrainedMaps/FrictionMaps/BoundConstrainedMDPOperatorQL.h"
#endif

#include <iostream>

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactOperator>& impact_operator, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_operator != nullptr )
  {
    const std::string name{ impact_operator->name() };
    StringUtilities::serialize( name, output_stream );
    impact_operator->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serialize( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<FrictionOperator>& friction_operator, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( friction_operator != nullptr )
  {
    const std::string friction_operator_name{ friction_operator->name() };
    StringUtilities::serialize( friction_operator_name, output_stream );
    friction_operator->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serialize( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<FrictionSolver>& friction_solver, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( friction_solver != nullptr )
  {
    const std::string friction_solver_name{ friction_solver->name() };
    StringUtilities::serialize( friction_solver_name, output_stream );
    friction_solver->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serialize( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactMap>& impact_map, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_map != nullptr )
  {
    // Currently, only one variant of impact map is supported
    const std::string imap_name{ "impact_map" };
    StringUtilities::serialize( imap_name, output_stream );
    impact_map->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serialize( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactFrictionMap>& impact_friction_map, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_friction_map != nullptr )
  {
    const std::string ifmap_name{ impact_friction_map->name() };
    StringUtilities::serialize( ifmap_name, output_stream );
    impact_friction_map->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serialize( null_string, output_stream );
  }
}


std::unique_ptr<ImpactOperator> ConstrainedMapUtilities::deserializeImpactOperator( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<ImpactOperator> impact_operator{ nullptr };

  // Read in the name of the operator
  const std::string impact_operator_name{ StringUtilities::deserialize( input_stream ) };
  // Read in the operator
  if( "gr" == impact_operator_name )
  {
    impact_operator.reset( new GROperator{ input_stream } );
  }
  else if( "grr" == impact_operator_name )
  {
    impact_operator.reset( new GRROperator{ input_stream } );
  }
  else if( "gauss_seidel" == impact_operator_name )
  {
    impact_operator.reset( new GaussSeidelOperator{ input_stream } );
  }
  #ifdef QL_FOUND
  else if( "lcp_ql" == impact_operator_name )
  {
    impact_operator.reset( new LCPOperatorQL{ input_stream } );
  }
  #endif
  #ifdef IPOPT_FOUND
  else if( "lcp_ipopt" == impact_operator_name )
  {
    impact_operator.reset( new LCPOperatorIpopt{ input_stream } );
  }
  #endif
  else if( "NULL" == impact_operator_name )
  {
    impact_operator.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for impact operator: " << impact_operator_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return impact_operator;
}

std::unique_ptr<FrictionOperator> ConstrainedMapUtilities::deserializeFrictionOperator( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<FrictionOperator> friction_operator{ nullptr };

  const std::string friction_operator_name{ StringUtilities::deserialize( input_stream ) };
  #ifdef QL_FOUND
  if( "linear_mdp_ql" == friction_operator_name )
  {
    friction_operator.reset( new LinearMDPOperatorQL{ input_stream } );
  }
  else if( "bound_constrained_mdp_ql" == friction_operator_name )
  {
    friction_operator.reset( new BoundConstrainedMDPOperatorQL{ input_stream } );
  }
  else
  #endif
  #ifdef IPOPT_FOUND
  if( "smooth_mdp_ipopt" == friction_operator_name )
  {
    friction_operator.reset( new SmoothMDPOperatorIpopt{ input_stream } );
  }
  else
  #endif
  if( "NULL" == friction_operator_name )
  {
    friction_operator.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for friction operator: " << friction_operator_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return friction_operator;
}

std::unique_ptr<FrictionSolver> ConstrainedMapUtilities::deserializeFrictionSolver( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<FrictionSolver> friction_solver{ nullptr };

  const std::string friction_solver_name{ StringUtilities::deserialize( input_stream ) };
  if( "staggered_projections" == friction_solver_name )
  {
    friction_solver.reset( new StaggeredProjections{ input_stream } );
  }
  else if( "sobogus" == friction_solver_name )
  {
    friction_solver.reset( new Sobogus{ input_stream } );
  }
  else if( "NULL" == friction_solver_name )
  {
    friction_solver.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for friction solver: " << friction_solver_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return friction_solver;
}

std::unique_ptr<ImpactMap> ConstrainedMapUtilities::deserializeImpactMap( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<ImpactMap> impact_map{ nullptr };

  const std::string impact_map_name{ StringUtilities::deserialize( input_stream ) };
  if( "impact_map" == impact_map_name )
  {
    impact_map.reset( new ImpactMap{ input_stream } );
  }
  else if( "NULL" == impact_map_name )
  {
    impact_map.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for impact map: " << impact_map_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return impact_map;
}

std::unique_ptr<ImpactFrictionMap> ConstrainedMapUtilities::deserializeImpactFrictionMap( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<ImpactFrictionMap> impact_friction_map{ nullptr };

  const std::string impact_friction_name{ StringUtilities::deserialize( input_stream ) };
  if( "geometric_impact_friction_map" == impact_friction_name )
  {
    impact_friction_map.reset( new GeometricImpactFrictionMap{ input_stream } );
  }
  else if( "stabilized_impact_friction_map" == impact_friction_name )
  {
    impact_friction_map.reset( new StabilizedImpactFrictionMap{ input_stream } );
  }
  else if( "NULL" == impact_friction_name )
  {
    impact_friction_map.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for impact friction map: " << impact_friction_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return impact_friction_map;
}
