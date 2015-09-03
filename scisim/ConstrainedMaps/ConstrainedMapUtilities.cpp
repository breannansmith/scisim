// ConstrainedMapUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "ConstrainedMapUtilities.h"

#include "SCISim/StringUtilities.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/LinearMDPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/LinearMDPOperatorQL.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/RestrictedSampleMDPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/BoundConstrainedMDPOperatorQL.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/BoundConstrainedMDPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/SmoothMDPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/GROperator.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/GRROperator.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/LCPOperatorIpopt.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/LCPOperatorQL.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/GaussSeidelOperator.h"
#include "SCISim/ConstrainedMaps/GeometricImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/StabilizedImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "SCISim/ConstrainedMaps/ImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/StaggeredProjections.h"
#include "SCISim/ConstrainedMaps/Sobogus.h"
#include "SCISim/HDF5File.cpp"
#include "SCISim/CompileDefinitions.h"
#include "SCISim/Timer/TimeUtils.h"

#include <ostream>
#include <istream>
#include <iostream>

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactOperator>& impact_operator, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_operator != nullptr )
  {
    const std::string name{ impact_operator->name() };
    StringUtilities::serializeString( name, output_stream );
    impact_operator->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serializeString( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<FrictionOperator>& friction_operator, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( friction_operator != nullptr )
  {
    const std::string friction_operator_name{ friction_operator->name() };
    StringUtilities::serializeString( friction_operator_name, output_stream );
    friction_operator->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serializeString( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<FrictionSolver>& friction_solver, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( friction_solver != nullptr )
  {
    const std::string friction_solver_name{ friction_solver->name() };
    StringUtilities::serializeString( friction_solver_name, output_stream );
    friction_solver->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serializeString( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactMap>& impact_map, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_map != nullptr )
  {
    // Currently, only one variant of impact map is supported
    const std::string imap_name{ "impact_map" };
    StringUtilities::serializeString( imap_name, output_stream );
    impact_map->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serializeString( null_string, output_stream );
  }
}

void ConstrainedMapUtilities::serialize( const std::unique_ptr<ImpactFrictionMap>& impact_friction_map, std::ostream& output_stream )
{
  assert( output_stream.good() );
  if( impact_friction_map != nullptr )
  {
    const std::string ifmap_name{ impact_friction_map->name() };
    StringUtilities::serializeString( ifmap_name, output_stream );
    impact_friction_map->serialize( output_stream );
  }
  else
  {
    const std::string null_string{ "NULL" };
    StringUtilities::serializeString( null_string, output_stream );
  }
}


std::unique_ptr<ImpactOperator> ConstrainedMapUtilities::deserializeImpactOperator( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<ImpactOperator> impact_operator{ nullptr };

  // Read in the name of the operator
  const std::string impact_operator_name{ StringUtilities::deserializeString( input_stream ) };
  // Read in the operator
  if( "lcp_ipopt" == impact_operator_name )
  {
    impact_operator.reset( new LCPOperatorIpopt{ input_stream } );
  }
  else if( "lcp_ql" == impact_operator_name )
  {
    impact_operator.reset( new LCPOperatorQL{ input_stream } );
  }
  else if( "gr" == impact_operator_name )
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

  const std::string friction_operator_name{ StringUtilities::deserializeString( input_stream ) };
  if( "linear_mdp_ipopt" == friction_operator_name )
  {
    friction_operator.reset( new LinearMDPOperatorIpopt{ input_stream } );
  }
  else if( "linear_mdp_ql" == friction_operator_name )
  {
    friction_operator.reset( new LinearMDPOperatorQL{ input_stream } );
  }
  else if( "restricted_sample_linear_mdp_ipopt" == friction_operator_name )
  {
    friction_operator.reset( new RestrictedSampleMDPOperatorIpopt{ input_stream } );
  }
  else if( "bound_constrained_mdp_ql" == friction_operator_name )
  {
    friction_operator.reset( new BoundConstrainedMDPOperatorQL{ input_stream } );
  }
  else if( "smooth_mdp_ipopt" == friction_operator_name )
  {
    friction_operator.reset( new SmoothMDPOperatorIpopt{ input_stream } );
  }
  else if( "bound_constrained_mdp_operator_ipopt" == friction_operator_name )
  {
    friction_operator.reset( new BoundConstrainedMDPOperatorIpopt{ input_stream } );
  }
  else if( "NULL" == friction_operator_name )
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

  const std::string friction_solver_name{ StringUtilities::deserializeString( input_stream ) };
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

  const std::string impact_map_name{ StringUtilities::deserializeString( input_stream ) };
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

  const std::string impact_friction_name{ StringUtilities::deserializeString( input_stream ) };
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
