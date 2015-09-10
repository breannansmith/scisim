// BoundConstrainedMDPOperatorIpopt.cpp
//
// Breannan Smith
// Last updated: 09/08/2015

#include "BoundConstrainedMDPOperatorIpopt.h"

#include "FrictionOperatorUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/ConstrainedMaps/IpoptUtilities.h"
#include "scisim/Math/MathUtilities.h"
#include "scisim/StringUtilities.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FischerBurmeisterBoundConstrained.h"

#include <iostream>

#ifdef IPOPT_FOUND
#include "IpIpoptCalculatedQuantities.hpp"
#include "IpIpoptData.hpp"
#include "IpTNLPAdapter.hpp"
#include "IpOrigIpoptNLP.hpp"
#endif

BoundConstrainedMDPOperatorIpopt::BoundConstrainedMDPOperatorIpopt( const std::vector<std::string>& linear_solvers, const scalar& tol )
: m_linear_solver_order( linear_solvers )
, m_tol( tol )
{
  assert( m_tol > 0.0 );

  // Verify that the user provided a valid linear solver option
  if( m_linear_solver_order.empty() )
  {
    std::cerr << "No linear solver provided to BoundConstrainedMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( IpoptUtilities::containsDuplicates( m_linear_solver_order ) )
  {
    std::cerr << "Duplicate linear solvers provided to BoundConstrainedMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  for( const std::string& solver_name : m_linear_solver_order )
  {
    if( !IpoptUtilities::linearSolverSupported( solver_name ) )
    {
      std::cerr << "Invalid linear solver provided to BoundConstrainedMDPOperatorIpopt: " << solver_name << ". Exiting." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }
}

BoundConstrainedMDPOperatorIpopt::BoundConstrainedMDPOperatorIpopt( std::istream& input_stream )
: m_linear_solver_order( Utilities::deserializeVectorCustomType( StringUtilities::deserializeString, input_stream ) )
, m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( !m_linear_solver_order.empty() );
  assert( m_tol > 0.0 );
}

BoundConstrainedMDPOperatorIpopt::BoundConstrainedMDPOperatorIpopt( const BoundConstrainedMDPOperatorIpopt& other )
: m_linear_solver_order( other.m_linear_solver_order )
, m_tol( other.m_tol )
{}

#ifdef IPOPT_FOUND
static void createIpoptApplication( const scalar& tol, Ipopt::SmartPtr<Ipopt::IpoptApplication>& ipopt_app )
{
  ipopt_app = IpoptApplicationFactory();

  // Initialize IPOPT
  if( ipopt_app->Initialize() != Ipopt::Solve_Succeeded )
  {
    std::cerr << "Error, failed to initialize Ipopt in BoundConstrainedMDPOperatorIpopt::createIpoptApplication. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // Set options
  assert( typeid(scalar) == typeid(double) );
  assert( tol > 0.0 );
  ipopt_app->Options()->SetNumericValue( "tol", tol );
  ipopt_app->Options()->SetIntegerValue( "print_level", 0 );
  ipopt_app->Options()->SetStringValue( "sb", "yes" ); // Don't print an Ipopt banner
  //ipopt_app->Options()->SetStringValue( "derivative_test", "second-order" );
  ipopt_app->Options()->SetStringValue( "hessian_constant", "yes" );
  ipopt_app->Options()->SetStringValue( "jac_c_constant", "yes" );
  ipopt_app->Options()->SetStringValue( "jac_d_constant", "yes" );
  //ipopt_app->Options()->SetStringValue( "mehrotra_algorithm", "yes" );
  #ifndef NDEBUG
  ipopt_app->Options()->SetStringValue( "check_derivatives_for_naninf", "yes" );
  #endif
}
#endif

void BoundConstrainedMDPOperatorIpopt::flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda )
{  
#ifdef IPOPT_FOUND
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  // Create the Ipopt-based QP solver
  // Use built in termination, for now
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new BoundConstrainedMDPNLP{ Q, beta, false, FischerBurmeisterBoundConstrained{ m_tol, ( mu.array() * alpha.array() ).matrix() } } };
  BoundConstrainedMDPNLP& qp_nlp{ *static_cast<BoundConstrainedMDPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // Linear term in the objective
  assert( D.rows() == v0.size() ); assert( D.cols() == gdotD.size() );
  qp_nlp.A() = D.transpose() * v0 + gdotD;

  // Bounds on the inequality constraints
  assert( mu.size() == alpha.size() ); assert( ( mu.array() >= 0.0 ).all() ); assert( ( alpha.array() >= -1.0e-8 ).all() );
  qp_nlp.C() = ( mu.array() * alpha.array() ).matrix();

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
      std::cerr << "Failed to solve friction QP in BoundConstrainedMDPOperatorIpopt::flow with Ipopt and linear solver " << solver_name << ": ";
      std::cerr << IpoptUtilities::ipoptReturnStatusToString( solve_status ) << std::endl;
      // If we exhausted all of the user-specified solvers
      if( solver_name == m_linear_solver_order.back() )
      {
        std::cerr << "Exhausted all linear solver options in BoundConstrainedMDPOperatorIpopt::flow. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }

  assert( qp_nlp.lambda().size() == alpha.size() );
  lambda = qp_nlp.lambda();

#else
  std::cerr << " Error, please rebuild with Ipopt support before executing BoundConstrainedMDPOperatorIpopt::flow." << std::endl;
  std::cerr << "            Ipopt can be obtained via: https://projects.coin-or.org/Ipopt" << std::endl;
  std::exit( EXIT_FAILURE );
#endif
}

int BoundConstrainedMDPOperatorIpopt::numFrictionImpulsesPerNormal() const
{
  return 1;
}

void BoundConstrainedMDPOperatorIpopt::formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.rows() == v.size() ); assert( D.cols() == int( K.size() ) );
  FrictionOperatorUtilities::formGeneralizedFrictionBasis( q, v, K, 1, D, drel );
}

std::string BoundConstrainedMDPOperatorIpopt::name() const
{
  return "bound_constrained_mdp_operator_ipopt";
}

std::unique_ptr<FrictionOperator> BoundConstrainedMDPOperatorIpopt::clone() const
{
  return std::unique_ptr<FrictionOperator>{ new BoundConstrainedMDPOperatorIpopt{ *this } };
}

void BoundConstrainedMDPOperatorIpopt::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeVectorCustomType( m_linear_solver_order, StringUtilities::serializeString, output_stream );
  Utilities::serializeBuiltInType( m_tol, output_stream );
}

void BoundConstrainedMDPOperatorIpopt::solveQP( const QPTerminationOperator& termination_operator, const SparseMatrixsc& Minv, const SparseMatrixsc& D, const VectorXs& b, const VectorXs& c, VectorXs& beta, VectorXs& lambda, scalar& achieved_tol ) const
{
  assert( b.size() == D.cols() ); assert( b.size() == c.size() ); assert( b.size() == beta.size() ); assert( b.size() == lambda.size() ); assert( Minv.rows() == Minv.cols() ); assert( Minv.rows() == D.rows() );

#ifdef IPOPT_FOUND
  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  const SparseMatrixsc Q{ D.transpose() * Minv * D };

  // Create the Ipopt-based QP solver
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new BoundConstrainedMDPNLP{ Q, beta, true, termination_operator } };
  BoundConstrainedMDPNLP& qp_nlp{ *static_cast<BoundConstrainedMDPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // Linear term in the objective
  qp_nlp.A() = b;

  // Bounds on the inequality constraints
  qp_nlp.C() = c;

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
      std::cerr << "Failed to solve friction QP in BoundConstrainedMDPOperatorIpopt::solveQP with Ipopt and linear solver " << solver_name << ": ";
      std::cerr << IpoptUtilities::ipoptReturnStatusToString( solve_status ) << std::endl;
      // If we exhausted all of the user-specified solvers
      if( solver_name == m_linear_solver_order.back() )
      {
        std::cerr << "Exhausted all linear solver options in BoundConstrainedMDPOperatorIpopt::solveQP. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }

  lambda = qp_nlp.lambda();

  achieved_tol = qp_nlp.achievedTolerance();

#else
  std::cerr << " Error, please rebuild with Ipopt support before executing BoundConstrainedMDPOperatorIpopt::solveQP." << std::endl;
  std::cerr << "            Ipopt can be obtained via: https://projects.coin-or.org/Ipopt" << std::endl;
  std::exit(EXIT_FAILURE);
#endif
}

bool BoundConstrainedMDPOperatorIpopt::isLinearized() const
{
  return false;
}







#ifdef IPOPT_FOUND

BoundConstrainedMDPNLP::BoundConstrainedMDPNLP( const SparseMatrixsc& Q, VectorXs& beta, const bool use_custom_termination, const QPTerminationOperator& termination_operator )
: m_Q( Q )
, m_A()
, m_C()
, m_beta( beta )
, m_lambda()
, m_solve_return_status()
, m_use_custom_termination( use_custom_termination )
, m_termination_operator( termination_operator )
, m_achieved_tolerance( SCALAR_INFINITY )
{}

BoundConstrainedMDPNLP::~BoundConstrainedMDPNLP()
{}

bool BoundConstrainedMDPNLP::get_nlp_info( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );

  n = m_A.size();
  m = 0; // All constraints are bound constraints
  nnz_jac_g = 0;
  nnz_h_lag = MathUtilities::nzLowerTriangular( m_Q );
  index_style = TNLP::C_STYLE;

  return true;
}

bool BoundConstrainedMDPNLP::get_bounds_info( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u )
{
  assert( typeid( Ipopt::Number ) == typeid( scalar ) );
  assert( n == m_A.size() );
  assert( n == m_C.size() );
  assert( m == 0 );
  assert( ( m_C.array() >= 0.0 ).all() );

  assert( x_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_l, n } = - m_C;

  assert( x_u != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_u, n } = m_C;

  return true;
}

// TODO: Why aren't z_L, z_U, and lambda nullptr ?
bool BoundConstrainedMDPNLP::get_starting_point( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda )
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

bool BoundConstrainedMDPNLP::eval_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( n == m_Q.rows() );
  assert( x != nullptr );

  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  obj_value = 0.5 * x_map.transpose() * m_Q * x_map + m_A.dot( x_map );

  return true;
}

bool BoundConstrainedMDPNLP::eval_grad_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
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

bool BoundConstrainedMDPNLP::eval_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g )
{
  assert( m == 0 );

  std::cerr << "BoundConstrainedMDPNLP::eval_g called with m == 0. This is unexpected behavior." << std::endl;
  //std::exit( EXIT_FAILURE );

  return true;
}

bool BoundConstrainedMDPNLP::eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( n == m_Q.rows() );
  assert( m == 0 );
  assert( nele_jac == 0 );

  // This function should only get called in the mode that requests the sparsity structure
  assert( x == nullptr );
  assert( iRow != nullptr );
  assert( jCol != nullptr );
  assert( values == nullptr );

  //std::cerr << " BoundConstrainedMDPNLP::eval_jac_g not needed. Perhaps an options was set incorrectly? Exiting." << std::endl;
  //std::exit( EXIT_FAILURE );

  return true;
}

bool BoundConstrainedMDPNLP::eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( n == m_Q.rows() );
  assert( m == 0 );

  // Note: Ipopt requires the elements on and below the diagonal
  if( values == nullptr )
  {
    assert( x == nullptr );
    assert( lambda == nullptr );
    assert( iRow != nullptr );
    assert( jCol != nullptr );
    assert( typeid(Ipopt::Index) == typeid(int) );
    #ifndef NDEBUG
      Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >( iRow, nele_hess ).setConstant( -1 );
      Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >( jCol, nele_hess ).setConstant( -1 );
    #endif

    const int nnz = MathUtilities::sparsityPatternLowerTriangular( m_Q, iRow, jCol );
    assert( nnz == nele_hess );
    Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
  }
  else
  {
    assert( x != nullptr );
    assert( iRow == nullptr );
    assert( jCol == nullptr );
    assert( typeid(Ipopt::Number) == typeid(scalar) );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ values, nele_hess }.setConstant( SCALAR_NAN );
    #endif

    const int nnz = MathUtilities::valuesLowerTriangular( m_Q, values );
    assert( nnz == nele_hess );
    Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ values, nele_hess } *= obj_factor;
  }

  return true;
}

bool BoundConstrainedMDPNLP::intermediate_callback( Ipopt::AlgorithmMode mode, Ipopt::Index iter, Ipopt::Number obj_value, Ipopt::Number inf_pr, Ipopt::Number inf_du, Ipopt::Number mu, Ipopt::Number d_norm, Ipopt::Number regularization_size, Ipopt::Number alpha_du, Ipopt::Number alpha_pr, Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq )
{
  // If the user did not request a custom termination, exit
  if( !m_use_custom_termination ) return true;

  Ipopt::TNLPAdapter* tnlp_adapter = nullptr;
  if( ip_cq != nullptr )
  {
    Ipopt::OrigIpoptNLP* orignlp;
    orignlp = dynamic_cast<Ipopt::OrigIpoptNLP*>( GetRawPtr( ip_cq->GetIpoptNLP() ) );
    if( orignlp != nullptr )
    {
      tnlp_adapter = dynamic_cast<Ipopt::TNLPAdapter*>( GetRawPtr( orignlp->nlp() ) );
    }
  }

  if( tnlp_adapter != nullptr )
  {
    tnlp_adapter->ResortX( *ip_data->curr()->x(), m_beta.data() );
    const VectorXs y{ m_Q * m_beta + m_A };
    const scalar current_tol{ m_termination_operator( m_beta, y ) };
    if( current_tol <= m_termination_operator.tol() )
    {
      m_achieved_tolerance = current_tol;
      return false;
    }
  }

  return true;
}

void BoundConstrainedMDPNLP::finalize_solution( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq )
{
  m_solve_return_status = status;

  m_beta = Eigen::Map< const Eigen::Matrix<Ipopt::Number,Eigen::Dynamic,1> >{ x, n };
  assert( ( m_beta.array() <= m_C.array() ).all() ); assert( ( m_beta.array() >= -m_C.array() ).all() );

  const Eigen::Map< const Eigen::Array<Ipopt::Number,Eigen::Dynamic,1> > z_L_map{ z_L, n };
  assert( ( z_L_map.array() >= 0.0 ).all() );
  const Eigen::Map< const Eigen::Array<Ipopt::Number,Eigen::Dynamic,1> > z_U_map{ z_U, n };
  assert( ( z_U_map.array() >= 0.0 ).all() );

  // Sanity check the constraint multipliers
  #ifndef NDEBUG
  for( int i = 0; i < n; ++i )
  {
    if( !( z_L_map( i ) <= 6.0e-5 || z_U_map( i ) <= 6.0e-5 || m_C( i ) <= 6.0e-5 ) )
    {
      std::cerr << i << " : " << z_L_map( i ) << " " << z_U_map( i ) << " " << m_C( i ) << std::endl;
    }
    assert( z_L_map(i) <= 6.0e-5 || z_U_map(i) <= 6.0e-5 || m_C(i) <= 6.0e-5 );
  }
  #endif

  m_lambda = z_L_map.max( z_U_map );

  if( m_use_custom_termination )
  {
    const VectorXs y{ m_Q * m_beta + m_A };
    m_achieved_tolerance = m_termination_operator( m_beta, y );
  }
}

Ipopt::SolverReturn BoundConstrainedMDPNLP::getReturnStatus() const
{
  return m_solve_return_status;
}

#endif
