// SmoothMDPOperatorIpopt.cpp
//
// Breannan Smith
// Last updated: 10/23/2015

#include "SmoothMDPOperatorIpopt.h"

#include "FrictionOperatorUtilities.h"

#include "scisim/ConstrainedMaps/IpoptUtilities.h"
#include "scisim/Math/MathUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/StringUtilities.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FischerBurmeisterSmooth.h"

#include <iostream>

#ifndef NDEBUG
#include <typeinfo>
#endif

#ifdef IPOPT_FOUND
#include "IpIpoptCalculatedQuantities.hpp"
#include "IpIpoptData.hpp"
#include "IpTNLPAdapter.hpp"
#include "IpOrigIpoptNLP.hpp"
#endif

SmoothMDPOperatorIpopt::SmoothMDPOperatorIpopt( const std::vector<std::string>& linear_solvers, const scalar& tol )
: m_linear_solver_order( linear_solvers )
, m_tol( tol )
{
  assert( m_tol > 0.0 );

  // Verify that the user provided a valid linear solver option
  if( m_linear_solver_order.empty() )
  {
    std::cerr << "No linear solver provided to SmoothMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( IpoptUtilities::containsDuplicates( m_linear_solver_order ) )
  {
    std::cerr << "Duplicate linear solvers provided to SmoothMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  for( const std::string& solver_name : m_linear_solver_order )
  {
    if( !IpoptUtilities::linearSolverSupported( solver_name ) )
    {
      std::cerr << "Invalid linear solver provided to SmoothMDPOperatorIpopt: " << solver_name << ". Exiting." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }
}

SmoothMDPOperatorIpopt::SmoothMDPOperatorIpopt( std::istream& input_stream )
: m_linear_solver_order( Utilities::deserializeVectorCustomType( StringUtilities::deserializeString, input_stream ) )
, m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( !m_linear_solver_order.empty() );
  assert( m_tol > 0.0 );
}

SmoothMDPOperatorIpopt::SmoothMDPOperatorIpopt( const SmoothMDPOperatorIpopt& other )
: m_linear_solver_order( other.m_linear_solver_order )
, m_tol( other.m_tol )
{
  assert( !m_linear_solver_order.empty() );
  assert( m_tol > 0.0 );
}

int SmoothMDPOperatorIpopt::numFrictionImpulsesPerNormal() const
{
  return 2;
}

std::string SmoothMDPOperatorIpopt::name() const
{
  return "smooth_mdp_ipopt";
}

std::unique_ptr<FrictionOperator> SmoothMDPOperatorIpopt::clone() const
{
  return std::unique_ptr<FrictionOperator>{ new SmoothMDPOperatorIpopt{ *this } };
}

void SmoothMDPOperatorIpopt::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeVectorCustomType( m_linear_solver_order, StringUtilities::serializeString, output_stream );
  Utilities::serializeBuiltInType( m_tol, output_stream );
}

bool SmoothMDPOperatorIpopt::isLinearized() const
{
  return false;
}

#ifdef IPOPT_FOUND
static void createIpoptApplication( const scalar& tol, Ipopt::SmartPtr<Ipopt::IpoptApplication>& ipopt_app )
{
  ipopt_app = IpoptApplicationFactory();

  // Initialize IPOPT
  if( ipopt_app->Initialize() != Ipopt::Solve_Succeeded )
  {
    std::cerr << "Error, failed to initialize Ipopt in SmoothMDPOperatorIpopt::createIpoptApplication. Exiting" << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // Set options
  assert( typeid(scalar) == typeid(double) );
  assert( tol > 0.0 );
  ipopt_app->Options()->SetNumericValue( "tol", tol );
  ipopt_app->Options()->SetIntegerValue( "print_level", 0 );
  ipopt_app->Options()->SetStringValue( "sb", "yes" ); // Don't print an Ipopt banner
  //ipopt_app->Options()->SetStringValue( "derivative_test", "second-order" );
  ipopt_app->Options()->SetStringValue( "hessian_constant", "no" );
  ipopt_app->Options()->SetStringValue( "jac_c_constant", "yes" );
  ipopt_app->Options()->SetStringValue( "jac_d_constant", "no" );
  //ipopt_app->Options()->SetStringValue( "mehrotra_algorithm", "yes" );
#ifndef NDEBUG
  ipopt_app->Options()->SetStringValue( "check_derivatives_for_naninf", "yes" );
#endif
}
#endif

void SmoothMDPOperatorIpopt::flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda )
{
#ifdef IPOPT_FOUND
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  // Create the Ipopt-based QP solver
  // Use built in termination, for now
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new SmoothMDPNLP{ Q, beta, false, FischerBurmeisterSmooth{ m_tol, ( mu.array() * alpha.array() ).matrix() } } };
  SmoothMDPNLP& qp_nlp{ *static_cast<SmoothMDPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // Linear term in the objective
  assert( D.rows() == v0.size() ); assert( D.cols() == gdotD.size() );
  qp_nlp.A() = D.transpose() * v0 + gdotD;

  // Bounds on the inequality constraints
  qp_nlp.C() = ( mu.array() * mu.array() * alpha.array() * alpha.array() ).matrix();

  // Backup beta, in case we need to fall back on another solver
  const VectorXs beta0{ beta };

  for( const std::string& solver_name : m_linear_solver_order )
  {
    // Reset the initial guess
    beta = beta0;

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
        std::cerr << "Exhausted all linear solver options in SmoothMDPOperatorIpopt::flow. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }

  // Rescale lambda to values we would have gotten if constraints were enforced with square roots
  assert( lambda.size() == alpha.size() );
  {
    const VectorXs vrel{ Q * beta + qp_nlp.A() };
    for( int lambda_idx = 0; lambda_idx < lambda.size(); ++lambda_idx )
    {
      lambda( lambda_idx ) = vrel.segment<2>( 2 * lambda_idx ).norm();
    }
  }

  // Check the optimality conditions
  // TODO: Replace with single call to min-map functional
  //#ifndef NDEBUG
  //{
  //  VectorXs friction_disk_constraints( alpha.size() );
  //  for( int con_num = 0; con_num < alpha.size(); ++con_num )
  //  {
  //    friction_disk_constraints( con_num ) = mu( con_num ) * alpha( con_num ) - beta.segment<2>( 2 * con_num ).norm();
  //  }
  //  assert( ( lambda.array() >= 0.0 ).all() );
  //  assert( ( friction_disk_constraints.array() >= - 100000.0 * m_tol ).all() );
  //  for( int con_num = 0; con_num < alpha.size(); ++con_num )
  //  {
  //    assert( fabs( lambda( con_num ) ) <= 100000.0 * m_tol || fabs( friction_disk_constraints( con_num ) ) <= 100000.0 * m_tol );
  //  }
  //  // || friction_disk_constraints ||_\inf can be big
  //  //std::cout << "Friction Constraints: " << friction_disk_constraints.transpose() << std::endl;
  //}
  //#endif

#else
  std::cerr << " Error, please rebuild with Ipopt support before executing SmoothMDPOperatorIpopt::flow." << std::endl;
  std::cerr << "            Ipopt can be obtained via: https://projects.coin-or.org/Ipopt" << std::endl;
  std::exit( EXIT_FAILURE );
#endif
}

void SmoothMDPOperatorIpopt::solveQP( const QPTerminationOperator& termination_operator, const SparseMatrixsc& Minv, const SparseMatrixsc& D, const VectorXs& b, const VectorXs& c, VectorXs& beta, VectorXs& lambda, scalar& achieved_tol ) const
{
#ifdef IPOPT_FOUND
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  const SparseMatrixsc Q{ D.transpose() * Minv * D };

  // Create the Ipopt-based QP solver
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new SmoothMDPNLP( Q, beta, true, termination_operator ) };
  SmoothMDPNLP& qp_nlp{ *static_cast<SmoothMDPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // Linear term in the objective
  qp_nlp.A() = b;

  // Bounds on the inequality constraints
  qp_nlp.C() = c.array() * c.array();

  // Backup beta, in case we need to fall back on another solver
  const VectorXs beta0{ beta };

  for( const std::string& solver_name : m_linear_solver_order )
  {
    // Reset the initial guess
    beta = beta0;

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
        std::cerr << "Exhausted all linear solver options in SmoothMDPOperatorIpopt::solveQP. Exiting." << std::endl;
      }
    }
    else
    {
      break;
    }
  }

  // Rescale lambda to values we would have gotten if constraints were enforced with square roots
  {
    const VectorXs vrel{ Q * beta + qp_nlp.A() };
    for( int lambda_idx = 0; lambda_idx < lambda.size(); ++lambda_idx )
    {
      lambda( lambda_idx ) = vrel.segment<2>( 2 * lambda_idx ).norm();
    }
  }

  achieved_tol = qp_nlp.achievedTolerance();

  // TODO: Sanity check the solution here
#else
  std::cerr << " Error, please rebuild with Ipopt support before executing SmoothMDPOperatorIpopt::solveQP." << std::endl;
  std::cerr << "            Ipopt can be obtained via: https://projects.coin-or.org/Ipopt" << std::endl;
  std::exit( EXIT_FAILURE );
#endif
}


#ifdef IPOPT_FOUND

SmoothMDPNLP::SmoothMDPNLP( const SparseMatrixsc& Q, VectorXs& beta, const bool use_custom_termination, const QPTerminationOperator& termination_operator )
: m_Q( Q )
, m_A()
, m_C()
, m_beta( beta )
, m_diagonal_indices()
, m_solve_return_status()
, m_use_custom_termination( use_custom_termination )
, m_termination_operator( termination_operator )
, m_achieved_tolerance( SCALAR_INFINITY )
{}

SmoothMDPNLP::~SmoothMDPNLP()
{}

bool SmoothMDPNLP::get_nlp_info( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );

  // Number of friction impulses
  assert( m_beta.size() == m_A.size() );
  n = m_A.size();

  // Number of constraints -- one per contact/pair-of-impulses
  assert( n % 2 == 0 );
  m = n / 2;

  nnz_jac_g = n;
  nnz_h_lag = MathUtilities::nzLowerTriangular( m_Q );
  index_style = TNLP::C_STYLE;

  return true;
}

bool SmoothMDPNLP::get_bounds_info( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( n == m_A.size() );
  assert( m == m_C.size() );

  // TODO: Could add an upper and lower bound to each beta... might it help?
  assert( x_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_l, n }.setConstant( -2e19 );
  assert( x_u != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_u, n }.setConstant( 2e19 );

  // TODO: Could add a lower bound to g... might it help?
  assert( g_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ g_l, m }.setConstant( -2e19 );
  
  assert( g_u != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ g_u, m } = m_C;

  return true;
}

// TODO: Why aren't z_L, z_U, and lambda nullptr ?
bool SmoothMDPNLP::get_starting_point( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( init_x );
  assert( x != nullptr );
  assert( !init_z );
  //assert( z_L == nullptr ); assert( z_U == nullptr );
  assert( !init_lambda );
  //assert( lambda == nullptr );

  assert( m_beta.size() == n );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x, n } = m_beta;

  return true;
}

bool SmoothMDPNLP::eval_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( x != nullptr );
  
  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  assert( m_Q.rows() == m_Q.cols() ); assert( m_Q.rows() == x_map.size() ); assert( m_A.size() == x_map.size() );
  obj_value = 0.5 * x_map.transpose() * m_Q * x_map + m_A.dot( x_map );

  return true;
}

bool SmoothMDPNLP::eval_grad_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( x != nullptr );
  assert( grad_f != nullptr );

  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > grad_map{ grad_f, n };

  assert( m_Q.rows() == m_Q.cols() ); assert( m_Q.rows() == x_map.size() ); assert( m_A.size() == x_map.size() );
  grad_map = m_Q * x_map + m_A;

  return true;
}

// TODO: Could re-write this with Eigen column wise stuff if we wanted
bool SmoothMDPNLP::eval_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( 2 * m == n );

  const Eigen::Map< const Eigen::Array< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  assert( x_map.size() % 2 == 0 );
  Eigen::Map< Eigen::Array< Ipopt::Number, Eigen::Dynamic, 1 > > g_map{ g, m };
  assert( x_map.size() == 2 * g_map.size() );

  for( int i = 0; i < m; ++i )
  {
    g_map( i ) = x_map( 2 * i + 0 ) * x_map( 2 * i + 0 ) + x_map( 2 * i + 1 ) * x_map( 2 * i + 1 );
  }

  assert( ( g_map >= 0.0 ).all() );

  return true;
}

bool SmoothMDPNLP::eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( 2 * m == n );
  assert( nele_jac == n );

  // Provide the sparsity structure
  if( values == nullptr )
  {
    assert( iRow != nullptr );
    assert( jCol != nullptr );

    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > > rowMap{ iRow, nele_jac };
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > > colMap{ jCol, nele_jac };

    for( Ipopt::Index i = 0; i < m; ++i )
    {
      rowMap.segment<2>( 2 * i ).setConstant( i );
    }
    for( Ipopt::Index i = 0; i < nele_jac; ++i )
    {
      colMap( i ) = i;
    }
  }
  // Provide the value of the Jacobian
  else
  {
    assert( x != nullptr );
    assert( values != nullptr );
    assert( iRow == nullptr );
    assert( jCol == nullptr );
    assert( typeid(Ipopt::Number) == typeid(scalar) );

    const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > xMap{ x, nele_jac };
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > valueMap{ values, nele_jac };

    valueMap = 2.0 * xMap;
  }

  return true;
}

bool SmoothMDPNLP::eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( 2 * m == n );

  // Note: Ipopt requires the elements on and below the diagonal
  if( values == nullptr )
  {
    assert( x == nullptr );
    assert( lambda == nullptr );
    assert( iRow != nullptr );
    assert( jCol != nullptr );
    assert( typeid(Ipopt::Index) == typeid(int) );
    {
      const int nnz{ MathUtilities::sparsityPatternLowerTriangular( m_Q, iRow, jCol ) };
      assert( nnz == nele_hess );
      Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
    }

    m_diagonal_indices.resize( n );
    for( int i = 0; i < nele_hess; ++i )
    {
      // If the entry is a diagonal element
      if( iRow[i] == jCol[i] )
      {
        m_diagonal_indices[iRow[i]] = i;
      }
    }
  }
  else
  {
    assert( x != nullptr );
    assert( lambda != nullptr );
    assert( iRow == nullptr );
    assert( jCol == nullptr );
    assert( typeid(Ipopt::Number) == typeid(scalar) );
    assert( Ipopt::Index(m_diagonal_indices.size()) == n );
    {
      const int nnz{ MathUtilities::valuesLowerTriangular( m_Q, values ) };
      assert( nnz == nele_hess );
      Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
    }
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >( values, nele_hess ) *= obj_factor;

    const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > lambda_map{ lambda, m };
    // TODO: These aren't here, yuck!
    const Eigen::Map< const Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > > row_map{ iRow, nele_hess };
    const Eigen::Map< const Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > > col_map{ jCol, nele_hess };

    // For each constraint
    for( int i = 0; i < m; ++i )
    {
      values[ m_diagonal_indices[ 2 * i + 0 ] ] += 2.0 * lambda_map( i );
      values[ m_diagonal_indices[ 2 * i + 1 ] ] += 2.0 * lambda_map( i );
    }
  }

  return true;
}

void SmoothMDPNLP::finalize_solution( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq )
{
  m_solve_return_status = status;
  m_beta = Eigen::Map< const Eigen::Matrix<Ipopt::Number,1,Eigen::Dynamic> >{ x, n };

  if( m_use_custom_termination )
  {
    const VectorXs y{ m_Q * m_beta + m_A };
    m_achieved_tolerance = m_termination_operator( m_beta, y );
  }
}

Ipopt::SolverReturn SmoothMDPNLP::getReturnStatus() const
{
  return m_solve_return_status;
}

#endif
