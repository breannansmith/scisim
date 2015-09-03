// LinearMDPOperatorQL.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "LinearMDPOperatorQL.h"

#include "FrictionOperatorUtilities.h"
#include "SCISim/StringUtilities.h"
#include "SCISim/Utilities.h"

#include <iostream>

#ifndef NDEBUG
#include <typeinfo>
#endif

// TODO: Move to a shared QL utility file
extern "C"
{
  void ql_( int* m, int* me, int* mmax,
            int* n, int* nmax,
            int* mnn,
            double* c, double* d,
            double* a, double* b,
            double* xl, double* xu,
            double* x, double* u,
            double* eps,
            int* mode, int* iout, int* ifail, int* iprint,
            double* war, int* lwar,
            int* iwar, int* liwar );
}

// TODO: Move into a QL support class
static std::string QLReturnStatustoString( const int status )
{
  if( 0 == status )
  {
    return "The optimality conditions are satisfied";
  }
  else if( 1 == status )
  {
    return "The algorithm has been stopped after too many MAXIT iterations (40*(N+M)";
  }
  else if( 2 == status )
  {
    return "Termination accuracy insufficient to satisfy convergence criterion";
  }
  else if( 3 == status )
  {
    return "Internal inconsistency of QL, division by zero";
  }
  else if( 5 == status )
  {
    return "Length of a working array is too short";
  }
  else if( 100 < status )
  {
    return "Constraints are inconsistent and IFAIL=100+ICON, where ICON denotes a constraint causing the conflict: " + StringUtilities::convertToString( status );
  }

  std::cerr << "Unhandled error message in LCPOperatorQL::QLReturnStatusToString. Probably a bug." << std::endl;
  std::exit( EXIT_FAILURE );
}

LinearMDPOperatorQL::LinearMDPOperatorQL( const int disk_samples, const scalar& eps )
: m_disk_samples( disk_samples )
, m_tol( eps )
{
  assert( m_disk_samples >= 1 );
  assert( m_tol >= 0.0 );
}

LinearMDPOperatorQL::LinearMDPOperatorQL( std::istream& input_stream )
: m_disk_samples( Utilities::deserialize<int>(input_stream ) )
, m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_disk_samples >= 1 );
  assert( m_tol >= 0.0 );
}

LinearMDPOperatorQL::LinearMDPOperatorQL( const LinearMDPOperatorQL& other )
: m_disk_samples( other.m_disk_samples )
, m_tol( other.m_tol )
{
  assert( m_disk_samples >= 1 );
  assert( m_tol >= 0.0 );
}

static int solveQP( const scalar& tol, MatrixXXsc& C, VectorXs& c, MatrixXXsc& A, VectorXs& b, VectorXs& beta, VectorXs& lambda )
{
  #ifdef QL_FOUND
  assert( typeid(scalar) == typeid(double) );  // QL only supports doubles
  assert( C.rows() == C.cols() ); assert( C.rows() == c.size() ); assert( C.rows() == A.cols() );
  assert( A.rows() == b.size() ); assert( C.rows() == beta.size() ); assert( lambda.size() == b.size() );

  // Inequality constraints
  int m = b.size();
  // No equality constraints
  int me = 0;
  // Row size of matrix containing linear constraints
  int mmax = A.rows();

  // Number of degrees of freedom.
  int n = C.rows();
  int nmax = n;
  int mnn = m + n + n;

  // Impose non-negativity constraints on all variables
  Eigen::VectorXd xl = Eigen::VectorXd::Zero( nmax );
  Eigen::VectorXd xu = Eigen::VectorXd::Constant( nmax, std::numeric_limits<double>::infinity() );

  // u will contain the constraint multipliers
  Eigen::VectorXd u( mnn );

  // Status of the solve
  int ifail = -1;
  // Use the built-in cholesky decomposition
  int mode = 1;
  // Some fortran output stuff
  int iout = 0;
  // 1 => print output, 0 => silent
  int iprint = 1;

  // Working space
  int lwar = 3 * nmax * nmax / 2 + 10 * nmax + mmax + m + 2;
  Eigen::VectorXd war( lwar );
  // Additional working space
  int liwar = n;
  Eigen::VectorXi iwar( liwar );

  assert( A.rows() == m ); assert( A.cols() == n );

  {
    scalar tol_local = tol;
    ql_( &m,
         &me,
         &mmax,
         &n,
         &nmax,
         &mnn,
         C.data(),
         c.data(),
         A.data(),
         b.data(),
         xl.data(),
         xu.data(),
         beta.data(),
         u.data(),
         &tol_local,
         &mode,
         &iout,
         &ifail,
         &iprint,
         war.data(),
         &lwar,
        iwar.data(),
         &liwar );
  }

  lambda = u.segment( 0, m );

  return ifail;
  #else
  std::cerr << " Error, please rebuild with QL support before executing FrictionOperatorQL::solveQP." << std::endl;
  std::cerr << "            QL can be obtained via: http://www.ai7.uni-bayreuth.de/ql.htm" << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void LinearMDPOperatorQL::flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda )
{
  // Total number of constraints
  const int num_constraints = alpha.size();
  // Total number of friction impulses
  const int num_impulses = m_disk_samples * alpha.size();

  // Quadratic term in the objective: 1/2 x^T C x
  assert( Q.rows() == Q.cols() ); assert( Q.rows() == num_impulses );
  MatrixXXsc C{ Q };
  assert( ( C - C.transpose() ).lpNorm<Eigen::Infinity>() < 1.0e-6 );

  // Linear term in the objective: c^T x
  assert( D.rows() == v0.size() ); assert( D.cols() == gdotD.size() );
  VectorXs c{ D.transpose() * v0 + gdotD }; // No 2 as QL puts a 1/2 in front of the quadratic term

  // Linear inequality constraint matrix
  MatrixXXsc A;
  {
    SparseMatrixsc E( num_impulses, num_constraints );
    FrictionOperatorUtilities::formLinearFrictionDiskConstraint( m_disk_samples, E );
    A = -E.transpose();
  }
  
  // Bounds on the inequality constraints: A^T x + b >= 0
  assert( mu.size() == alpha.size() );
  VectorXs b{ ( mu.array() * alpha.array() ).matrix() };
  
  // Use QL to solve the QP
  assert( beta.size() == num_impulses ); assert( lambda.size() == alpha.size() );
  const int status = solveQP( m_tol, C, c, A, b, beta, lambda );
  
  // Check for problems
  if( 0 != status )
  {
    std::cout << "Warning, failed to solve QP in FrictionOperatorQL::flow: " << QLReturnStatustoString(status) << "." << std::endl;
  }

  // Check the optimality conditions
  // TODO: Move these checks to a standard min-map functional
  #ifndef NDEBUG
  {
    // \beta should be positive
    assert( ( beta.array() >= - m_tol ).all() );
    // \lambda should be positive
    assert( ( lambda.array() >= - m_tol ).all() );

    // vrel >= -E^T \lambda
    const ArrayXs vrel = ( Q * beta + c ).array();
    SparseMatrixsc E( num_impulses, num_constraints );
    FrictionOperatorUtilities::formLinearFrictionDiskConstraint( m_disk_samples, E );
    const ArrayXs Elam = ( E * lambda ).array();
    assert( ( vrel + Elam >= - m_tol * ( vrel.abs() + Elam.abs() ) - 10000.0 * m_tol ).all() );

    // \mu \alpha >= E^T beta
    const ArrayXs mualpha = mu.array() * alpha.array();
    const ArrayXs Ebet = ( E.transpose() * beta ).array();
    assert( ( mualpha + Ebet >= - m_tol * ( mualpha.abs() + Ebet.abs() ) - m_tol ).all() );

    // \beta \perp vrel + E \lambda
    //const ArrayXs rhs_top = vrel + Elam;
    //assert( ( ( beta.array() * rhs_top ).abs() < ( m_eps * beta.array().abs().max( rhs_top.abs() ) + 100000.0 * m_eps + 1.0e-2 ) ).all() );

    // \lambda \perp \mu \alpha - E^T \beta
    const ArrayXs rhs_bot = mualpha - Ebet;
    assert( ( ( lambda.array() * rhs_bot ).abs() < ( 100000.0 * m_tol * lambda.array().abs().max( rhs_bot.abs() ) + m_tol ) ).all() );
  }
  #endif
}

int LinearMDPOperatorQL::numFrictionImpulsesPerNormal() const
{
  return m_disk_samples;
}

void LinearMDPOperatorQL::formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.rows() == v.size() );
  assert( D.cols() == int( m_disk_samples * K.size() ) );
  FrictionOperatorUtilities::formGeneralizedFrictionBasis( q, v, K, m_disk_samples, D, drel );
}

std::string LinearMDPOperatorQL::name() const
{
  return "linear_mdp_ql";
}

std::unique_ptr<FrictionOperator> LinearMDPOperatorQL::clone() const
{
  return std::unique_ptr<FrictionOperator>{ new LinearMDPOperatorQL{ *this } };
}

void LinearMDPOperatorQL::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serializeBuiltInType( m_disk_samples, output_stream );
  Utilities::serializeBuiltInType( m_tol, output_stream );
}

bool LinearMDPOperatorQL::isLinearized() const
{
  return true;
}
