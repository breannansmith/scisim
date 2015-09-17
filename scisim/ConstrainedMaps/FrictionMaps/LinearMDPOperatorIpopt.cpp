// LinearMDPOperatorIpopt.cpp
//
// Breannan Smith
// Last updated: 09/08/2015

#include "LinearMDPOperatorIpopt.h"

#include "FrictionOperatorUtilities.h"

#include "scisim/ConstrainedMaps/IpoptUtilities.h"
#include "scisim/Math/MathUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/StringUtilities.h"

#ifndef NDEBUG
#include <typeinfo>
#endif

#include <fstream>
#include <iostream>

LinearMDPOperatorIpopt::LinearMDPOperatorIpopt( const int disk_samples, const std::vector<std::string>& linear_solvers, const scalar& tol )
: m_disk_samples( disk_samples )
, m_linear_solver_order( linear_solvers )
, m_tol( tol )
{
  assert( m_disk_samples > 0 );
  assert( m_tol > 0.0 );

  // Verify that the user provided a valid linear solver option
  if( m_linear_solver_order.empty() )
  {
    std::cerr << "No linear solver provided to LinearMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( IpoptUtilities::containsDuplicates( m_linear_solver_order ) )
  {
    std::cerr << "Duplicate linear solvers provided to LinearMDPOperatorIpopt. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  for( const std::string& solver_name : m_linear_solver_order )
  {
    if( !IpoptUtilities::linearSolverSupported( solver_name ) )
    {
      std::cerr << "Invalid linear solver provided to LinearMDPOperatorIpopt: " << solver_name << ". Exiting." << std::endl;
      std::exit( EXIT_FAILURE );
    }
  }
}

LinearMDPOperatorIpopt::LinearMDPOperatorIpopt( std::istream& input_stream )
: m_disk_samples( Utilities::deserialize<int>( input_stream ) )
, m_linear_solver_order( Utilities::deserializeVectorCustomType( StringUtilities::deserializeString, input_stream ) )
, m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( input_stream.good() );
  assert( !m_linear_solver_order.empty() );
  assert( m_tol > 0.0 );
}

LinearMDPOperatorIpopt::LinearMDPOperatorIpopt( const LinearMDPOperatorIpopt& other )
: m_disk_samples( other.m_disk_samples )
, m_linear_solver_order( other.m_linear_solver_order )
, m_tol( other.m_tol )
{
  assert( !m_linear_solver_order.empty() );
  assert( m_tol > 0.0 );
}

#ifdef IPOPT_FOUND
static void createIpoptApplication( const scalar& tol, Ipopt::SmartPtr<Ipopt::IpoptApplication>& ipopt_app )
{
  ipopt_app = IpoptApplicationFactory();

  // Initialize IPOPT
  const Ipopt::ApplicationReturnStatus status = ipopt_app->Initialize();
  if( status != Ipopt::Solve_Succeeded )
  {
    std::cerr << "Error, failed to initialize Ipopt in LinearMDPOperatorIpopt::createIpoptApplication. Exiting" << std::endl;
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

void LinearMDPOperatorIpopt::flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda )
{
#ifdef IPOPT_FOUND
  // Total number of constraints
  const int num_constraints{ static_cast<int>( alpha.size() ) };
  // Total number of friction impulses
  const int num_impulses{ static_cast<int>( m_disk_samples * alpha.size() ) };

  Ipopt::SmartPtr<Ipopt::IpoptApplication> ipopt_app;
  createIpoptApplication( m_tol, ipopt_app );

  // Create the Ipopt-based QP solver
  Ipopt::SmartPtr<Ipopt::TNLP> ipopt_problem{ new LinearMDPNLP{ Q, beta } };
  LinearMDPNLP& qp_nlp{ *static_cast<LinearMDPNLP*>( GetRawPtr( ipopt_problem ) ) };

  // Linear term in the objective
  assert( D.rows() == v0.size() ); assert( D.cols() == gdotD.size() );
  qp_nlp.A() = D.transpose() * v0 + gdotD;

  // Linear inequality constraint matrix
  qp_nlp.E().resize( num_impulses, num_constraints );
  FrictionOperatorUtilities::formLinearFrictionDiskConstraint( m_disk_samples, qp_nlp.E() );

  // Bounds on the inequality constraints
  assert( mu.size() == alpha.size() );
  qp_nlp.C() = ( mu.array() * alpha.array() ).matrix();

  // Backup beta, in case we need to fall back on another solver
  const VectorXs beta0{ beta };

  for( const std::string& solver_name : m_linear_solver_order )
  {
    assert( qp_nlp.beta().size() == num_impulses );
    qp_nlp.beta() = beta0;

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
        std::cerr << "Exhausted all linear solver options in LinearMDPOperatorIpopt::flow. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      break;
    }
  }

  lambda = qp_nlp.lambda();
  assert( lambda.size() == alpha.size() );

  // Check the optimality conditions
  // TODO: Replace these massive checks with a simple min-map functional
  #ifndef NDEBUG
  {
    // \beta should be positive
    assert( ( beta.array() >= 0.0 ).all() );
    // \lambda should be positive
    assert( ( lambda.array() >= -m_tol ).all() );

    // vrel >= -E^T \lambda
    const ArrayXs vrel{ ( Q * beta + qp_nlp.A() ).array() };
    const ArrayXs Elam{ ( qp_nlp.E() * lambda ).array() };
    assert( ( vrel + Elam >= - m_tol * ( vrel.abs() + Elam.abs() ) - 100.0 * m_tol ).all() );

    // \mu \alpha >= E^T beta
    const ArrayXs mualpha{ mu.array() * alpha.array() };
    const ArrayXs Ebet{ ( qp_nlp.E().transpose() * beta ).array() };
    assert( ( mualpha + Ebet >= -m_tol * ( mualpha.abs() + Ebet.abs() ) - m_tol ).all() );

    // \beta \perp vrel + E \lambda
    const ArrayXs rhs_top{ vrel + Elam };
    //std::cout << ( ( beta.array() * rhs_top ).abs() - ( m_tol * beta.array().abs().max( rhs_top.abs() ) + 10000.0 * m_tol ) ).transpose()  << std::endl;
    assert( ( ( beta.array() * rhs_top ).abs() < ( m_tol * beta.array().abs().max( rhs_top.abs() ) + 100000.0 * m_tol ) ).all() );

    // \lambda \perp \mu \alpha - E^T \beta
    const ArrayXs rhs_bot{ mualpha - Ebet };
    assert( ( ( lambda.array() * rhs_bot ).abs() < ( 100000.0 * m_tol * lambda.array().abs().max( rhs_bot.abs() ) + m_tol ) ).all() );
  }
  #endif

#else
  std::cerr << " Error, please rebuild with Ipopt support before executing LinearMDPOperatorIpopt::flow." << std::endl;
  std::cerr << "            Ipopt can be obtained via: https://projects.coin-or.org/Ipopt" << std::endl;
  std::exit( EXIT_FAILURE );
#endif
}

int LinearMDPOperatorIpopt::numFrictionImpulsesPerNormal() const
{
  return m_disk_samples;
}

void LinearMDPOperatorIpopt::formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.rows() == v.size() );
  assert( D.cols() == int( m_disk_samples * K.size() ) );
  FrictionOperatorUtilities::formGeneralizedFrictionBasis( q, v, K, m_disk_samples, D, drel );
}

std::string LinearMDPOperatorIpopt::name() const
{
  return "linear_mdp_ipopt";
}

std::unique_ptr<FrictionOperator> LinearMDPOperatorIpopt::clone() const
{
  return std::unique_ptr<FrictionOperator>{ new LinearMDPOperatorIpopt{ *this } };
}

void LinearMDPOperatorIpopt::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeBuiltInType( m_disk_samples, output_stream );
  Utilities::serializeVectorCustomType( m_linear_solver_order, StringUtilities::serializeString, output_stream );
  Utilities::serializeBuiltInType( m_tol, output_stream );
}

bool LinearMDPOperatorIpopt::isLinearized() const
{
  return true;
}









#ifdef IPOPT_FOUND

LinearMDPNLP::LinearMDPNLP( const SparseMatrixsc& Q, VectorXs& beta )
: m_Q( Q )
, m_A()
, m_E()
, m_C()
, m_beta( beta )
, m_lambda()
, m_solve_return_status()
{}

LinearMDPNLP::~LinearMDPNLP()
{}

bool LinearMDPNLP::get_nlp_info( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, TNLP::IndexStyleEnum& index_style )
{
  assert( m_Q.rows() == m_Q.cols() );
  assert( m_A.size() == m_Q.rows() );
  assert( m_E.rows() == m_A.size() );
  assert( m_E.cols() == m_C.size() );
  assert( m_E.rows() >= m_E.cols() );
  assert( m_E.rows() % m_E.cols() == 0 );
  assert( m_E.rows() == m_E.nonZeros() );

  n = m_A.size();
  m = m_E.cols();
  nnz_jac_g = m_E.nonZeros();
  nnz_h_lag = MathUtilities::nzLowerTriangular( m_Q );
  index_style = TNLP::C_STYLE;

  return true;
}

// TODO: Figure out how to get infinity out of ipopt (1e19 is documented as infinity, but that could change...)
bool LinearMDPNLP::get_bounds_info( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( n == m_A.size() );
  assert( m == m_C.size() );

  // Lower bound on beta required; see Staggered Projections
  assert( x_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_l, n }.setZero();

  assert( x_u != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ x_u, n }.setConstant( 2e19 );

  assert( g_l != nullptr );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ g_l, m }.setConstant( -2e19 );

  // Upper bound required on affine constraint; see Staggered Projections
  assert( g_u != nullptr ); assert( m_C.size() == m );
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ g_u, m } = m_C;

  return true;
}

// TODO: Why aren't z_L, z_U, and lambda nullptr ?
bool LinearMDPNLP::get_starting_point( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda )
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

bool LinearMDPNLP::eval_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value )
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

bool LinearMDPNLP::eval_grad_f( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f )
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

bool LinearMDPNLP::eval_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g )
{
  assert( typeid(Ipopt::Number) == typeid(scalar) );
  assert( m_E.rows() == n );
  assert( m_E.cols() == m );

  const Eigen::Map< const Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > > x_map{ x, n };
  Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ g, m } = m_E.transpose() * x_map;

  return true;
}

bool LinearMDPNLP::eval_jac_g( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  // Provide the sparsity structure
  if( values == nullptr )
  {
    assert( iRow != nullptr );
    assert( jCol != nullptr );
    assert( nele_jac == m_E.nonZeros() );
    assert( typeid(Ipopt::Index) == typeid(int) );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ iRow, nele_jac }.setConstant( -1 );
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ jCol, nele_jac }.setConstant( -1 );
    #endif

    // We are loading in the transpose so iRow and jCol are swapped here
    {
      const int nnz{ MathUtilities::sparsityPattern( m_E, jCol, iRow ) };
      assert( nnz == nele_jac );
      Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
    }

    #ifndef NDEBUG
    {
      for( int i = 0; i < nele_jac; ++i )
      {
        assert( jCol[i] == i );
      }
      assert( n % m == 0 );
      const int nsamples = n / m;
      for( int i = 0; i < m; ++i )
      {
        for( int j = nsamples * i; j < nsamples * i + nsamples; ++j )
        {
          assert( iRow[j] == i );
        }
      }
    }
    #endif
  }
  // Provide the value of the Jacobian
  else
  {
    assert( x != nullptr );
    assert( values != nullptr );
    assert( iRow == nullptr );
    assert( jCol == nullptr );
    assert( nele_jac == m_E.nonZeros() );
    assert( typeid(Ipopt::Number) == typeid(scalar) );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Number, Eigen::Dynamic, 1 > >{ values, nele_jac }.setConstant( SCALAR_NAN );
    #endif

    // TODO: Just set to 1!
    const int nnz{ MathUtilities::values( m_E, values ) };
    assert( nnz == nele_jac );
    Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode

    #ifndef NDEBUG
    {
      for( int i = 0; i < nele_jac; ++i )
      {
        assert( values[i] == 1.0 );
      }
    }
    #endif
  }

  return true;
}

bool LinearMDPNLP::eval_h( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values )
{
  // Note: Ipopt requires the elements on and below the diagonal
  if( values == nullptr )
  {
    assert( x == nullptr );
    assert( lambda == nullptr );
    assert( iRow != nullptr );
    assert( jCol != nullptr );
    assert( typeid(Ipopt::Index) == typeid(int) );
    #ifndef NDEBUG
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ iRow, nele_hess }.setConstant( -1 );
    Eigen::Map< Eigen::Matrix< Ipopt::Index, Eigen::Dynamic, 1 > >{ jCol, nele_hess }.setConstant( -1 );
    #endif

    const int nnz = MathUtilities::sparsityPatternLowerTriangular( m_Q, iRow, jCol );
    assert( nnz == nele_hess );
    Utilities::ignoreUnusedVariable( nnz ); // To silence warnings in release mode
  }
  else
  {
    assert( x != nullptr );
    assert( lambda != nullptr );
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

void LinearMDPNLP::finalize_solution( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq )
{
  m_solve_return_status = status;

  m_beta = Eigen::Map< const Eigen::Matrix<Ipopt::Number,1,Eigen::Dynamic> >{ x, n };

  m_lambda = Eigen::Map< const Eigen::Matrix<Ipopt::Number,1,Eigen::Dynamic> >{ lambda, m };
}

Ipopt::SolverReturn LinearMDPNLP::getReturnStatus() const
{
  return m_solve_return_status;
}

#endif
