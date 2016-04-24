// rigidbody3d_cli.cpp
//
// Breannan Smith
// Last updated: 10/01/2015

#ifdef USE_PYTHON
#include <Python.h>
#endif

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstdint>
#include <getopt.h>

#include "scisim/StringUtilities.h"
#include "scisim/Math/MathDefines.h"
#include "scisim/Math/MathUtilities.h"
#include "scisim/Math/Rational.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactSolution.h"
#include "scisim/Timer/TimeUtils.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ConstrainedMapUtilities.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/CompileDefinitions.h"
#include "scisim/HDF5File.h"
#include "scisim/Utilities.h"
#include "scisim/PythonTools.h"

#include "rigidbody3d/RigidBody3DSim.h"
#include "rigidbody3d/PythonScripting.h"
#include "rigidbody3d/RigidBody3DUtilities.h"

#include "rigidbody3dutils/RigidBody3DSceneParser.h"
#include "rigidbody3dutils/RenderingState.h"


// TODO: 'Front-pad' the time so all output is same width
// TODO: Also print out frame ### / 100 or something

static RigidBody3DSim g_sim;
static unsigned g_iteration = 0;
static std::unique_ptr<UnconstrainedMap> g_unconstrained_map{ nullptr };
static Rational<std::intmax_t> g_dt;
static scalar g_end_time = SCALAR_NAN;
static std::unique_ptr<ImpactOperator> g_impact_operator{ nullptr };
static scalar g_CoR = SCALAR_NAN;
static std::unique_ptr<FrictionSolver> g_friction_solver{ nullptr };
static scalar g_mu = SCALAR_NAN;
static std::unique_ptr<ImpactFrictionMap> g_impact_friction_map{ nullptr };
static PythonScripting g_scripting;

static std::string g_output_dir_name;
static bool g_output_forces{ false };
// Number of timesteps between saves
static unsigned g_steps_per_save{ 0 };
// Number of saves that been conducted so far
static unsigned g_output_frame{ 0 };
static unsigned g_dt_string_precision{ 0 };
static unsigned g_save_number_width{ 0 };

static bool g_serialize_snapshots{ false };
static bool g_overwrite_snapshots{ true };

// Magic number to print in front of binary output to aid in debugging
static const unsigned MAGIC_BINARY_NUMBER{ 8675309 };

static std::string generateOutputConfigurationDataFileName( const std::string& prefix, const std::string& extension )
{
  std::stringstream ss;
  if( !g_output_dir_name.empty() )
  {
    ss << g_output_dir_name << "/";
  }
  ss << prefix << "_" << std::setfill('0') << std::setw( g_save_number_width ) << g_output_frame << "." << extension;
  return ss.str();
}

static void printCompileInfo( std::ostream& output_stream )
{
  output_stream << "Git Revision:     " << CompileDefinitions::GitSHA1 << std::endl;
  output_stream << "Build Mode:       " << CompileDefinitions::BuildMode << std::endl;
  output_stream << "C Compiler:       " << CompileDefinitions::CCompiler << std::endl;
  output_stream << "C++ Compiler:     " << CompileDefinitions::CXXCompiler << std::endl;
  #ifdef FORTRAN_FOUND
  output_stream << "Fortran Compiler: " << CompileDefinitions::FortranCompiler << std::endl;
  #endif
}

static unsigned computeTimestepDisplayPrecision( const Rational<std::intmax_t>& dt, const std::string& dt_string )
{
  if( dt_string.find( '.' ) != std::string::npos )
  {
    return unsigned( StringUtilities::computeNumCharactersToRight( dt_string, '.' ) );
  }
  else
  {
    std::string converted_dt_string;
    std::stringstream ss;
    ss << std::fixed << scalar( dt );
    ss >> converted_dt_string;
    return unsigned( StringUtilities::computeNumCharactersToRight( converted_dt_string, '.' ) );
  }
}

static std::string xmlFilePath( const std::string& xml_file_name )
{
  std::string path;
  std::string file_name;
  StringUtilities::splitAtLastCharacterOccurence( xml_file_name, path, file_name, '/' );
  if( file_name.empty() )
  {
    using std::swap;
    swap( path, file_name );
  }
  return path;
}

// TODO: Move all of the loaded state to local variables, only set globals when state is verified
static bool loadXMLScene( const std::string& xml_file_name )
{
  // Simulation data to load
  RigidBody3DState new_sim_state;
  std::string new_scripting_callback_name;

  // Attempt to load the scene
  {
    std::string new_dt_string;
    RenderingState UNUSED_rendering_state_UNUSED;

    const bool loaded_successfully{ RigidBody3DSceneParser::parseXMLSceneFile( xml_file_name, new_scripting_callback_name, new_sim_state, g_unconstrained_map, new_dt_string, g_dt, g_end_time, g_impact_operator, g_CoR, g_friction_solver, g_mu, g_impact_friction_map, UNUSED_rendering_state_UNUSED ) };
    if( !loaded_successfully )
    {
      return false;
    }

    g_dt_string_precision = computeTimestepDisplayPrecision( g_dt, new_dt_string );
  }

  g_sim.getState() = std::move( new_sim_state );
  g_sim.clearConstraintCache(); // <- TODO: probs not needed, but won't hurt... just assert that it is empty, instead

  PythonScripting new_scripting{ xmlFilePath( xml_file_name ), new_scripting_callback_name };
  swap( g_scripting, new_scripting );


  // User-provided start of simulation python callback
  g_scripting.setState( g_sim.getState() );
  g_scripting.setInitialIterate( g_iteration );
  g_scripting.startOfSimCallback();
  g_scripting.forgetState();

  return true;
}

static std::string generateSimulationTimeString()
{
  std::stringstream time_stream;
  time_stream << std::fixed << std::setprecision( g_dt_string_precision ) << g_iteration * scalar( g_dt );
  return time_stream.str();
}

static int saveState()
{
  #ifdef USE_HDF5
  // Generate a base filename
  const std::string output_file_name = generateOutputConfigurationDataFileName( "config", "h5" );

  // Print a status message with the simulation time and output number
  std::cout << "Saving state at time " << generateSimulationTimeString() << " to " << output_file_name;
  std::cout << "        " << TimeUtils::currentTime() << std::endl;

  // Save the simulation state
  try
  {
    HDF5File output_file{ output_file_name, HDF5AccessType::READ_WRITE };
    // Save the iteration and time step and time
    output_file.writeScalar( "", "timestep", scalar( g_dt ) );
    output_file.writeScalar( "", "iteration", g_iteration );
    output_file.writeScalar( "", "time", scalar( g_dt ) * g_iteration );
    // Save out the git hash
    output_file.writeString( "", "git_hash", CompileDefinitions::GitSHA1 );
    // Save the real time
    //output_file.writeString( "/run_stats", "real_time", TimeUtils::currentTime() );
    // Write out the simulation data
    g_sim.writeBinaryState( output_file );
  }
  catch( const std::string& error )
  {
    std::cerr << error << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
  #else
  std::cerr << "Error, state output requires HDF5 support." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

static int serializeSystem()
{
  // Generate a base filename
  const std::string serialized_file_name{ g_overwrite_snapshots ? "serial.bin" : generateOutputConfigurationDataFileName( "serial", "bin" ) };

  // Print a message to the user that the state is being written
  std::cout << "Serializing: " << generateSimulationTimeString() << " to " << serialized_file_name;
  std::cout << "        " << TimeUtils::currentTime() << std::endl;

  // Attempt to open the output file
  std::ofstream serial_stream{ serialized_file_name, std::ios::binary };
  if( !serial_stream.is_open() )
  {
    std::cerr << "Failed to open serialization file: " << serialized_file_name << std::endl;
    std::cerr << "Exiting." << std::endl;
    return EXIT_FAILURE;
  }

  // Write the magic number
  Utilities::serialize( MAGIC_BINARY_NUMBER, serial_stream );

  // Write the git revision
  {
    const std::string git_revision{ CompileDefinitions::GitSHA1 };
    StringUtilities::serialize( git_revision, serial_stream );
  }

  // Write the actual state
  g_sim.serialize( serial_stream );
  Utilities::serialize( g_iteration, serial_stream );
  RigidBody3DUtilities::serialize( g_unconstrained_map, serial_stream );
  Utilities::serialize( g_dt, serial_stream );
  Utilities::serialize( g_end_time, serial_stream );
  ConstrainedMapUtilities::serialize( g_impact_operator, serial_stream );
  Utilities::serialize( g_CoR, serial_stream );
  ConstrainedMapUtilities::serialize( g_friction_solver, serial_stream );
  Utilities::serialize( g_mu, serial_stream );
  ConstrainedMapUtilities::serialize( g_impact_friction_map, serial_stream );
  g_scripting.serialize( serial_stream );
  StringUtilities::serialize( g_output_dir_name, serial_stream );
  Utilities::serialize( g_output_forces, serial_stream );
  Utilities::serialize( g_steps_per_save, serial_stream );
  Utilities::serialize( g_output_frame, serial_stream );
  Utilities::serialize( g_dt_string_precision, serial_stream );
  Utilities::serialize( g_save_number_width, serial_stream );
  Utilities::serialize( g_serialize_snapshots, serial_stream );
  Utilities::serialize( g_overwrite_snapshots, serial_stream );

  return EXIT_SUCCESS;
}

static int deserializeSystem( const std::string& file_name )
{
  std::cout << "Loading serialized simulation state file: " << file_name << std::endl;

  // Attempt to open the input file
  std::ifstream serial_stream{ file_name, std::ios::binary };
  if( !serial_stream.is_open() )
  {
    std::cerr << "Failed to open serialization file: " << file_name << std::endl;
    std::cerr << "Exiting." << std::endl;
    return EXIT_FAILURE;
  }

  // Verify the magic number
  if( Utilities::deserialize<unsigned>( serial_stream ) != MAGIC_BINARY_NUMBER )
  {
    std::cerr << "File " << file_name << " does not appear to be a serialized 3D SCISim simulation. Exiting." << std::endl;
    return EXIT_FAILURE;
  }
  
  // Read the git revision
  {
    const std::string git_revision{ StringUtilities::deserialize( serial_stream ) };
    if( CompileDefinitions::GitSHA1 != git_revision )
    {
      std::cerr << "Warning, resuming from data file for a different git revision." << std::endl;
      std::cerr << "   Serialized Git Revision: " << git_revision << std::endl;
      std::cerr << "      Current Git Revision: " << CompileDefinitions::GitSHA1 << std::endl;
    }
    std::cout << "Git Revision: " << git_revision << std::endl;
  }

  g_sim.deserialize( serial_stream );
  g_iteration = Utilities::deserialize<unsigned>( serial_stream );
  g_unconstrained_map = RigidBody3DUtilities::deserializeUnconstrainedMap( serial_stream );
  g_dt = Utilities::deserialize<Rational<std::intmax_t>>( serial_stream );
  assert( g_dt.positive() );
  g_end_time = Utilities::deserialize<scalar>( serial_stream );
  assert( g_end_time > 0.0 );
  g_impact_operator = ConstrainedMapUtilities::deserializeImpactOperator( serial_stream );
  g_CoR = Utilities::deserialize<scalar>( serial_stream );
  assert( std::isnan(g_CoR) || g_CoR >= 0.0 ); assert( std::isnan(g_CoR) || g_CoR <= 1.0 );
  g_friction_solver = ConstrainedMapUtilities::deserializeFrictionSolver( serial_stream );
  g_mu = Utilities::deserialize<scalar>( serial_stream );
  assert( std::isnan(g_mu) || g_mu >= 0.0 );
  g_impact_friction_map = ConstrainedMapUtilities::deserializeImpactFrictionMap( serial_stream );
  {
    PythonScripting new_scripting{ serial_stream };
    swap( g_scripting, new_scripting );
  }
  g_output_dir_name = StringUtilities::deserialize( serial_stream );
  g_output_forces = Utilities::deserialize<bool>( serial_stream );
  g_steps_per_save = Utilities::deserialize<unsigned>( serial_stream );
  g_output_frame = Utilities::deserialize<unsigned>( serial_stream );
  g_dt_string_precision = Utilities::deserialize<unsigned>( serial_stream );
  g_save_number_width = Utilities::deserialize<unsigned>( serial_stream );
  g_serialize_snapshots = Utilities::deserialize<bool>( serial_stream );
  g_overwrite_snapshots = Utilities::deserialize<bool>( serial_stream );

  return EXIT_SUCCESS;
}

static int exportConfigurationData()
{
  assert( g_steps_per_save != 0 );
  if( g_iteration % g_steps_per_save == 0 )
  {
    if( !g_output_dir_name.empty() )
    {
      if( saveState() == EXIT_FAILURE )
      {
        return EXIT_FAILURE;
      }
    }
    if( g_serialize_snapshots )
    {
      if( serializeSystem() == EXIT_FAILURE )
      {
        return EXIT_FAILURE;
      }
    }
    ++g_output_frame;
  }
  return EXIT_SUCCESS;
}

#ifdef USE_HDF5
static std::string generateOutputConstraintForceDataFileName()
{
  std::stringstream ss;
  assert( g_output_frame > 0 );
  ss << g_output_dir_name << "/forces_" << std::setfill('0') << std::setw( g_save_number_width ) << g_output_frame - 1 << ".h5";
  return ss.str();
}
#endif

static int stepSystem()
{
  const unsigned next_iter = g_iteration + 1;

  HDF5File force_file;
  assert( g_steps_per_save != 0 );
  if( g_output_forces && g_iteration % g_steps_per_save == 0 )
  {
    #ifdef USE_HDF5
    assert( !g_output_dir_name.empty() );
    const std::string constraint_force_file_name = generateOutputConstraintForceDataFileName();
    std::cout << "Saving forces at time " << generateSimulationTimeString() << " to " << constraint_force_file_name << std::endl;
    try
    {
      force_file.open( constraint_force_file_name, HDF5AccessType::READ_WRITE );
      // Save the iteration and time step and time
      force_file.writeScalar( "", "timestep", scalar( g_dt ) );
      force_file.writeScalar( "", "iteration", g_iteration );
      force_file.writeScalar( "", "time", scalar( g_dt ) * g_iteration );
      // Save out the git hash
      force_file.writeString( "", "git_hash", CompileDefinitions::GitSHA1 );
      // Save the real time
      //force_file.writeString( "/run_stats", "real_time", TimeUtils::currentTime() );
    }
    catch( const std::string& error )
    {
      std::cerr << error << std::endl;
      return EXIT_FAILURE;
    }
    #else
    std::cerr << "Error, force output requires HDF5 support." << std::endl;
    std::exit( EXIT_FAILURE );
    #endif
  }

  if( g_unconstrained_map == nullptr && g_impact_operator == nullptr && g_friction_solver == nullptr && g_impact_friction_map == nullptr )
  {
    // Nothing to do
  }
  else if( g_unconstrained_map != nullptr && g_impact_operator == nullptr && g_friction_solver == nullptr && g_impact_friction_map == nullptr )
  {
    g_sim.flow( g_scripting, next_iter, g_dt, *g_unconstrained_map );
  }
  else if( g_unconstrained_map != nullptr && g_impact_operator != nullptr && g_friction_solver == nullptr && g_impact_friction_map == nullptr )
  {
    ImpactSolution impact_solution;
    if( force_file.is_open() )
    {
      g_sim.impactMap().exportForcesNextStep( impact_solution );
    }
    g_sim.flow( g_scripting, next_iter, g_dt, *g_unconstrained_map, *g_impact_operator, g_CoR );
    if( force_file.is_open() )
    {
      try
      {
        impact_solution.writeSolution( force_file );
      }
      catch( const std::string& error )
      {
        std::cerr << error << std::endl;
        return EXIT_FAILURE;
      }
    }
  }
  else if( g_unconstrained_map != nullptr && g_impact_operator == nullptr && g_friction_solver != nullptr && g_impact_friction_map != nullptr )
  {
    if( force_file.is_open() )
    {
      g_impact_friction_map->exportForcesNextStep( force_file );
    }
    g_sim.flow( g_scripting, next_iter, g_dt, *g_unconstrained_map, g_CoR, g_mu, *g_friction_solver, *g_impact_friction_map );
  }
  else
  {
    std::cerr << "Impossible code path hit in stepSystem. This is a bug. Exiting." << std::endl;
    return EXIT_FAILURE;
  }

  ++g_iteration;

  return exportConfigurationData();
}

static int executeSimLoop()
{
  if( exportConfigurationData() == EXIT_FAILURE )
  {
    return EXIT_FAILURE;
  }

  while( true )
  {
    // N.B. this will ocassionaly not trigger at the *exact* equal time due to floating point errors
    if( g_iteration * scalar( g_dt ) >= g_end_time )
    {
      // Take one final step to ensure we have force data for end time
      if( g_output_forces )
      {
        if( stepSystem() == EXIT_FAILURE )
        {
          return EXIT_FAILURE;
        }
      }
      // User-provided end of simulation python callback
      g_scripting.setState( g_sim.getState() );
      g_scripting.endOfSimCallback();
      g_scripting.forgetState();
      std::cout << "Simulation complete at time " << g_iteration * scalar( g_dt ) << ". Exiting." << std::endl;
      return EXIT_SUCCESS;
    }

    if( stepSystem() == EXIT_FAILURE )
    {
      return EXIT_FAILURE;
    }
  }
}

static void printUsage( const std::string& executable_name )
{
  std::cout << "Usage: " << executable_name << " xml_scene_file_name [options]" << std::endl;
  std::cout << "Options are:" << std::endl;
  std::cout << "   -h/--help                : prints this help message and exits" << std::endl;
  std::cout << "   -i/--impulses            : saves impulses in addition to configuration if an output directory is set" << std::endl;
  std::cout << "   -r/--resume file         : resumes the simulation from a serialized file" << std::endl;
  std::cout << "   -e/--end scalar          : overrides the end time specified in the scene file" << std::endl;
  std::cout << "   -o/--output_dir dir      : saves simulation state to the given directory" << std::endl;
  std::cout << "   -f/--frequency integer   : rate at which to save simulation data, in Hz; ignored if no output directory specified" << std::endl;
  std::cout << "   -s/--serialize_snapshots bool : save a bit identical, resumable snapshot; if 0 overwrites the snapshot each timestep, if 1 saves a new snapshot for each timestep" << std::endl;
}

static bool parseCommandLineOptions( int* argc, char*** argv, bool& help_mode_enabled, scalar& end_time_override, unsigned& output_frequency, std::string& serialized_file_name )
{
  const struct option long_options[] =
  {
    { "help", no_argument, nullptr, 'h' },
    { "impulses", no_argument, nullptr, 'i' },
    { "serialize_snapshots", required_argument, nullptr, 's' },
    { "resume", required_argument, nullptr, 'r' },
    { "end", required_argument, nullptr, 'e' },
    { "output_dir", required_argument, nullptr, 'o' },
    { "frequency", required_argument, nullptr, 'f' },
    { nullptr, 0, nullptr, 0 }
  };

  while( true )
  {
    int option_index = 0;
    const int c = getopt_long( *argc, *argv, "his:r:e:o:f:", long_options, &option_index );
    if( c == -1 )
    {
      break;
    }
    switch( c )
    {
      case 'h':
      {
        help_mode_enabled = true;
        break;
      }
      case 'i':
      {
        g_output_forces = true;
        break;
      }
      case 's':
      {
        g_serialize_snapshots = true;
        if( !StringUtilities::extractFromString( optarg, g_overwrite_snapshots ) )
        {
          std::cerr << "Failed to read value for argument for -s/--serialize_snapshots. Value must be a boolean." << std::endl;
          return false;
        }
        g_overwrite_snapshots = !g_overwrite_snapshots;
        break;
      }
      case 'r':
      {
        serialized_file_name = optarg;
        break;
      }
      case 'e':
      {
        if( !StringUtilities::extractFromString( optarg, end_time_override ) )
        {
          std::cerr << "Failed to read value for argument for -e/--end. Value must be a positive scalar." << std::endl;
          return false;
        }
        if( end_time_override <= 0 )
        {
          std::cerr << "Failed to read value for argument for -e/--end. Value must be a positive scalar." << std::endl;
          return false;
        }
        break;
      }
      case 'o':
      {
        g_output_dir_name = optarg;
        break;
      }
      case 'f':
      {
        if( !StringUtilities::extractFromString( optarg, output_frequency ) )
        {
          std::cerr << "Failed to read value for argument for -f/--frequency. Value must be an unsigned integer." << std::endl;
          return false;
        }
        break;
      }
      case '?':
      {
        return false;
      }
      default:
      {
        std::cerr << "This is a bug in the command line parser. Please file a report." << std::endl;
        return false;
      }
    }
  }
  
  return true;
}

#ifdef USE_PYTHON
static void exitCleanup()
{
  Py_Finalize();
}
#endif

int main( int argc, char** argv )
{
  // Command line options
  bool help_mode_enabled{ false };
  scalar end_time_override{ -1.0 };
  unsigned output_frequency{ 0 };
  std::string serialized_file_name;

  // Attempt to load command line options
  if( !parseCommandLineOptions( &argc, &argv, help_mode_enabled, end_time_override, output_frequency, serialized_file_name ) )
  {
    return EXIT_FAILURE;
  }

  // If the user requested help, print help and exit
  if( help_mode_enabled )
  {
    printUsage( argv[0] );
    return EXIT_SUCCESS;
  }

  // Check for impossible combinations of options
  if( g_output_forces && g_output_dir_name.empty() )
  {
    std::cerr << "Impulse output requires an output directory." << std::endl;
    return EXIT_FAILURE;
  }

  #ifdef USE_PYTHON
  // Initialize the Python interpreter
  Py_SetProgramName( argv[0] );
  Py_Initialize();

  // Initialize a callback that will close down the interpreter
  atexit( exitCleanup );

  // Allow subsequent Python commands to use the sys module
  PythonTools::pythonCommand( "import sys" );

  // Prevent Python from intercepting the interrupt signal
  PythonTools::pythonCommand( "import signal" );
  PythonTools::pythonCommand( "signal.signal( signal.SIGINT, signal.SIG_DFL )" );

  // Initialize the callbacks
  PythonScripting::initializeCallbacks();
  #endif

  if( !serialized_file_name.empty() )
  {
    if( deserializeSystem( serialized_file_name ) == EXIT_FAILURE )
    {
      return EXIT_FAILURE;
    }
    return executeSimLoop();
  }

  // The user must provide the path to an xml scene file
  if( argc != optind + 1 )
  {
    std::cerr << "Invalid arguments. Must provide a single xml scene file name." << std::endl;
    return EXIT_FAILURE;
  }

  // Attempt to load the user-provided scene
  if( !loadXMLScene( std::string{ argv[optind] } ) )
  {
    return EXIT_FAILURE;
  }

  // Override the default end time with the requested one, if provided
  if( end_time_override > 0.0 )
  {
    g_end_time = end_time_override;
  }

  // Compute the data output rate
  assert( g_dt.positive() );
  // If the user provided an output frequency
  if( output_frequency != 0 )
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( g_dt * std::intmax_t( output_frequency ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      std::cerr << "Timestep and output frequency do not yield an integer number of timesteps for data output. Exiting." << std::endl;
      return EXIT_FAILURE;
    }
    g_steps_per_save = unsigned( potential_steps_per_frame.numerator() );
  }
  // Otherwise default to dumping every frame
  else
  {
    g_steps_per_save = 1;
  }
  assert( g_end_time > 0.0 );
  g_save_number_width = MathUtilities::computeNumDigits( 1 + unsigned( ceil( g_end_time / scalar( g_dt ) ) ) / g_steps_per_save );

  printCompileInfo( std::cout );
  std::cout << "Geometry count: " << g_sim.state().ngeo() << std::endl;
  std::cout << "Body count: " << g_sim.state().nbodies() << std::endl;

  // If there are any intitial collisions, warn the user
  {
    std::map<std::string,unsigned> collision_counts;
    std::map<std::string,scalar> collision_depths;
    std::map<std::string,scalar> overlap_volumes;
    g_sim.computeNumberOfCollisions( collision_counts, collision_depths, overlap_volumes );
    assert( collision_counts.size() == collision_depths.size() ); assert( collision_counts.size() == overlap_volumes.size() );
    if( !collision_counts.empty() )
    {
      std::cout << "Warning, initial collisions detected (name : count : total_depth : total_volume):" << std::endl;
    }
    for( const auto& count_pair : collision_counts )
    {
      const std::string& constraint_name{ count_pair.first };
      const unsigned& constraint_count{ count_pair.second };
      assert( collision_depths.find( constraint_name ) != collision_depths.cend() );
      const scalar& constraint_depth{ collision_depths[constraint_name] };
      const scalar& constraint_volume{ overlap_volumes[constraint_name] };
      std::string depth_string;
      if( !std::isnan( constraint_depth ) )
      {
        depth_string = StringUtilities::convertToString( constraint_depth );
      }
      else
      {
        depth_string = "depth_computation_not_supported";
      }
      std::string volume_string;
      if( !std::isnan( constraint_volume ) )
      {
        volume_string = StringUtilities::convertToString( constraint_volume );
      }
      else
      {
        volume_string = "volume_computation_not_supported";
      }
      std::cout << "   " << constraint_name << " : " << constraint_count << " : " << depth_string << " : " << volume_string << std::endl;
    }
  }

  if( g_end_time == SCALAR_INFINITY )
  {
    std::cout << "No end time specified. Simulation will run indefinitely." << std::endl;
  }

  //scalar total_volume = 0.0;
  //for( int bdy_idx = 0; bdy_idx < g_sim.state().nbodies(); ++bdy_idx )
  //{
  //  total_volume += g_sim.state().getGeometryOfBody( bdy_idx ).volume();
  //}
  //std::cout << "Total volume: " << total_volume << std::endl;

  return executeSimLoop();
}
