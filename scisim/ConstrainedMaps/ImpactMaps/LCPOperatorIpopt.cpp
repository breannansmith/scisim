// LCPOperatorIpopt.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "LCPOperatorIpopt.h"

#include <iostream>

#include "scisim/StringUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/ConstrainedMaps/IpoptUtilities.h"

#ifndef NDEBUG
#include <typeinfo>
#endif
#include "IpIpoptCalculatedQuantities.hpp"
#include "IpIpoptData.hpp"
#include "IpTNLPAdapter.hpp"
#include "IpOrigIpoptNLP.hpp"
#include "scisim/ConstrainedMaps/ImpactMaps/FischerBurmeisterImpact.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/ConstrainedMaps/QPTerminationOperator.h"

LCPOperatorIpopt::LCPOperatorIpopt( const std::vector<std::string>& linear_solvers, const scalar& tol )
: m_linear_solver_order( linear_solvers )
, m_tol( tol )
{
  assert( m_tol > 0.0 );

  // Verify that the user provided a valid linear solver option
  if( m_linear_solver_order.empty() )
  {
    std::cerr << "No linear solver provided to LCPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( IpoptUtilities::containsDuplicates( m_linear_solver_order ) )
  {
    std::cerr << "Duplicate linear solvers provided to LCPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  for( const std::string& solver_name : m_linear_solver_order )
  {
    if( !IpoptUtilities::linearSolverSupported( solver_name ) )
    {
      std::cerr << "Invalid linear solver provided to LCPOperatorIpopt: " << solver_name << ". Exiting." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }
}

LCPOperatorIpopt::LCPOperatorIpopt( std::istream& input_stream )
: m_linear_solver_order( Utilities::deserializeVectorCustomType( StringUtilities::deserializeString, input_stream ) )
, m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( !m_linear_solver_order.empty() );
  assert( m_tol >= 0.0 );
}

static void createIpoptApplication( const scalar& tol, Ipopt::SmartPtr<Ipopt::IpoptApplication>& ipopt_app )
{
  ipopt_app = IpoptApplicationFactory();

  // Initialize IPOPT
  const Ipopt::ApplicationReturnStatus status = ipopt_app->Initialize();
  if( status != Ipopt::Solve_Succeeded )
  {
    std::cerr << "Error, failed to initialize Ipopt in LCPOperatorIpopt::createIpoptApplication. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // Set options
  assert( tol > 0.0 );
  ipopt_app->Options()->SetNumericValue( "tol", tol );
  ipopt_app->Options()->SetIntegerValue( "print_level", 0 );
  ipopt_app->Options()->SetStringValue( "sb", "yes" ); // Don't print an Ipopt banner
  //ipopt_app->Options()->SetStringValue( "derivative_test", "second-order" );
  ipopt_app->Options()->SetStringValue( "hessian_constant", "yes" );
  ipopt_app->Options()->SetStringValue( "jac_c_constant", "yes" );
  ipopt_app->Options()->SetStringValue( "jac_d_constant", "yes" );
  //ipopt_app->Options()->SetStringValue( "warm_start_init_point", "yes" );
  //ipopt_app->Options()->SetNumericValue( "warm_start_bound_push", 1.0e-9 );
  //ipopt_app->Options()->SetNumericValue( "warm_start_mult_bound_push", 1.0e-9 );
  //ipopt_app->Options()->SetStringValue( "mu_strategy", "adaptive" );
  //ipopt_app->Options()->SetStringValue( "mehrotra_algorithm", "yes" );
#ifndef NDEBUG
  ipopt_app->Options()->SetStringValue( "check_derivatives_for_naninf", "yes" );
#endif
}

void LCPOperatorIpopt::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  // Create the Ipopt-based QP solver
  assert( Q.rows() == Q.cols() );
  // Use built in termination, for now
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem = new QPNLP{ Q, false, FischerBurmeisterImpact{ m_tol } };
  QPNLP& qp_nlp{ *static_cast<QPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // A in A^T \alpha
  ImpactOperatorUtilities::computeLCPQPLinearTerm( N, nrel, CoR, v0, v0F, qp_nlp.A() );

  // Backup alpha, in case we need to fall back on another solver
  const VectorXs alpha0{ alpha };

  assert( N.cols() == nrel.size() ); assert( alpha.size() == nrel.size() );
  qp_nlp.setAlpha( &alpha );

  for( const std::string& solver_name : m_linear_solver_order )
  {
    // Reset the initial guess
    alpha = alpha0;

    // Set the current linear solver
    assert( !solver_name.empty() );
    ipopt_app->Options()->SetStringValue( "linear_solver", solver_name );
    // Try to solve the QP
    ipopt_app->OptimizeTNLP( ipopt_problem );
    const Ipopt::SolverReturn solve_status{ qp_nlp.getReturnStatus() };

    // If the solve failed
    if( solve_status != Ipopt::SUCCESS && solve_status != Ipopt::STOP_AT_ACCEPTABLE_POINT )
    {
      // Print an error message
      std::cerr << "Failed to solve QP with Ipopt and linear solver " << solver_name << ": ";
      std::cerr << IpoptUtilities::ipoptReturnStatusToString( solve_status ) << std::endl;
      // If we exhausted all of the user-specified solvers
      if( solver_name == m_linear_solver_order.back() )
      {
        std::cerr << "Exhausted all linear solver options in LCPOperatorIpopt::flow. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }
}

void LCPOperatorIpopt::solveQP( const QPTerminationOperator& termination_operator, const SparseMatrixsc& Minv, const SparseMatrixsc& N, const VectorXs& b, VectorXs& alpha, scalar& achieved_tol ) const
{
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  const SparseMatrixsc Q{ N.transpose() * Minv * N };

  // Create the Ipopt-based QP solver
  assert( Q.rows() == Q.cols() );
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new QPNLP{ Q, true, termination_operator } };
  QPNLP& qp_nlp{ *static_cast<QPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // A in A^T \alpha
  qp_nlp.A() = b;

  // Backup alpha, in case we need to fall back on another solver
  const VectorXs alpha0{ alpha };

  qp_nlp.setAlpha( &alpha );

  for( const std::string& solver_name : m_linear_solver_order )
  {
    // Reset the initial guess
    alpha = alpha0;

    // Set the current linear solver
    assert( !solver_name.empty() );
    ipopt_app->Options()->SetStringValue( "linear_solver", solver_name );
    // Try to solve the QP
    ipopt_app->OptimizeTNLP( ipopt_problem );
    const Ipopt::SolverReturn solve_status{ qp_nlp.getReturnStatus() };

    // If the solve failed
    if( solve_status != Ipopt::SUCCESS && solve_status != Ipopt::STOP_AT_ACCEPTABLE_POINT && solve_status != Ipopt::USER_REQUESTED_STOP )
    {
      // Print an error message
      std::cerr << "Failed to solve QP with Ipopt and linear solver " << solver_name << ": ";
      std::cerr << IpoptUtilities::ipoptReturnStatusToString( solve_status ) << std::endl;
      // If we exhausted all of the user-specified solvers
      if( solver_name == m_linear_solver_order.back() )
      {
        std::cerr << "Exhausted all linear solver options in LCPOperatorIpopt::solveQP. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }

  achieved_tol = qp_nlp.achievedTolerance();
}

std::string LCPOperatorIpopt::name() const
{
  return "lcp_ipopt";
}

std::unique_ptr<ImpactOperator> LCPOperatorIpopt::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new LCPOperatorIpopt{ m_linear_solver_order, m_tol } };
}

void LCPOperatorIpopt::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeVectorCustomType( m_linear_solver_order, StringUtilities::serializeString, output_stream );
  Utilities::serializeBuiltInType( m_tol, output_stream );
}





QPNLP::QPNLP( const SparseMatrixsc& Q, const bool use_custom_termination, const QPTerminationOperator& termination_operator )
: m_Q( Q )
, m_A()
, m_alpha( nullptr )
, m_solve_return_status( Ipopt::INTERNAL_ERROR )
, m_use_custom_termination( use_custom_termination )
, m_termination_operator( termination_operator )
, m_achieved_tolerance( SCALAR_INFINITY )
{}

QPNLP::~QPNLP()
{}

bool QPNLP::get_nlp_info( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );

  n = Ipopt::Index( m_A.size() );
  m = 0;
  nnz_jac_g = 0;
  nnz_h_lag = IpoptUtilities::nzLowerTriangular( m_Q );
  index_style = TNLP::C_STYLE;

  return true;
}

bool QPNLP::get_bounds_info( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u )
{
  static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( m == 0 );

  assert( x_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_l, n }.setZero();

  // TODO: Figure out how to get infinity out of ipopt (1e19 is documented as infinity, but that could change...)
  assert( x_u != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_u, n }.setConstant( 2e19 );

  return true;
}

// TODO: Why aren't z_L, z_U, and lambda nullptr ?
bool QPNLP::get_starting_point( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda )
{
  static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( m == 0 );

  if( init_x )
  {
    assert( x != nullptr ); assert( m_alpha != nullptr ); assert( m_alpha->size() == n );
    Eigen::Map<Eigen::Matrix<Ipopt::Number,Eigen::Dynamic,1>>{ x, n } = *m_alpha;
  }

  if( init_z )
  {
    assert( init_z ); assert( z_L != nullptr ); assert( z_U != nullptr );
    assert( m_alpha != nullptr ); assert( m_alpha->size() == n );
    assert( m_A.size() == n ); assert( m_Q.rows() == m_Q.cols() ); assert( m_Q.rows() == n );
    Eigen::Map<Eigen::Matrix<Ipopt::Number,Eigen::Dynamic,1>>{ z_L, n } = m_Q * (*m_alpha) + m_A;
    // Upper bounds should be unused
    #ifndef NDEBUG
    Eigen::Map<Eigen::Matrix<Ipopt::Number,Eigen::Dynamic,1>>{ z_U, n }.setConstant( SCALAR_NAN );
    #endif
  }

  // m == 0, so no need to initialize lambda

  return true;
}  

bool QPNLP::eval_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value )
{
  static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( x != nullptr );

  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  obj_value = 0.5 * x_map.transpose() * m_Q * x_map + m_A.dot( x_map );

  return true;
}  

bool QPNLP::eval_grad_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f )
{
  static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( x != nullptr );
  assert( grad_f != nullptr );

  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > grad_map{ grad_f, n };

  grad_map = m_Q * x_map + m_A;

  return true;
}

bool QPNLP::eval_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g )
{
  std::cerr << " QPNLP::eval_g not needed. Perhaps an options was set incorrectly? Exiting." << std::endl;
  std::exit( EXIT_FAILURE );
}

// TODO: A little wierd that this is getting called... email Ipopt people
bool QPNLP::eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );

  assert( n == m_Q.rows() );
  assert( m == 0 );
  assert( nele_jac == 0 );

  // This function should only get called in the mode that requests the sparsity structure
  assert( x == nullptr );
  assert( iRow != nullptr );
  assert( jCol != nullptr );
  assert( values == nullptr );

  return true;
}

bool QPNLP::eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( m == 0 );

  // Note: Ipopt requires the elements on and below the diagonal
  if( values == nullptr )
  {
    assert( x == nullptr );
    assert( lambda == nullptr );
    assert( iRow != nullptr );
    assert( jCol != nullptr );
    static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ iRow, nele_hess }.setConstant( -1 );
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ jCol, nele_hess }.setConstant( -1 );
    #endif

    IpoptUtilities::sparsityPatternLowerTriangular( m_Q, iRow, jCol );
  }
  else
  {
    assert( x != nullptr );
    //assert( lambda != nullptr );
    assert( iRow == nullptr );
    assert( jCol == nullptr );
    static_assert( std::is_same<Ipopt::Number,scalar>::value, "Ipopt's floating point type must be the same type as SCISim's scalar." );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ values, nele_hess }.setConstant( SCALAR_NAN );
    #endif

    IpoptUtilities::valuesLowerTriangular( m_Q, values );
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ values, nele_hess } *= obj_factor;
  }

  return true;
}

void QPNLP::finalize_solution( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( m_alpha != nullptr );
  assert( m_alpha->size() == m_A.size() );
  assert( n == m_A.size() );
  assert( m == 0 );

  m_solve_return_status = status;

  *m_alpha = Eigen::Map<const Eigen::Matrix<Ipopt::Number,1,Eigen::Dynamic>>{ x, n };

  if( m_use_custom_termination )
  {
    const VectorXs y{ m_Q * (*m_alpha) + m_A };
    m_achieved_tolerance = m_termination_operator( *m_alpha, y );
  }

  // Verify that the dual is in the ballpark
  #ifndef NDEBUG
  {
    const Eigen::Map<const Eigen::Matrix<Ipopt::Number,Eigen::Dynamic,1>> lower_dual{ z_L, n };
    const VectorXs computed_dual{ m_Q * (*m_alpha) + m_A };
    assert( ( lower_dual - computed_dual ).lpNorm<Eigen::Infinity>() <= 5.0e-6 );
  }
  #endif
}

void QPNLP::setAlpha( VectorXs* alpha )
{
  m_alpha = alpha;
}

Ipopt::SolverReturn QPNLP::getReturnStatus() const
{
  return m_solve_return_status;
}
