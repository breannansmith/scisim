// BoundConstrainedMDPOperatorQL.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "BoundConstrainedMDPOperatorQL.h"

#include "FrictionOperatorUtilities.h"
#include "SCISim/StringUtilities.h"
#include "SCISim/Utilities.h"

#include <iostream>
#ifndef NDEBUG
#include <typeinfo>
#endif

// TODO: Move to shared QL Utilities class
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

// TODO: Move to shared QL Utilities class
static std::string QLReturnStatusToString( const int status )
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

BoundConstrainedMDPOperatorQL::BoundConstrainedMDPOperatorQL( const scalar& tol )
: m_tol( tol )
{
  assert( m_tol > 0.0 );
}

BoundConstrainedMDPOperatorQL::BoundConstrainedMDPOperatorQL( std::istream& input_stream )
: m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_tol > 0.0 );
}

BoundConstrainedMDPOperatorQL::BoundConstrainedMDPOperatorQL( const BoundConstrainedMDPOperatorQL& other )
: m_tol( other.m_tol )
{
  assert( m_tol > 0.0 );
}

int solveQP( const scalar& tol, MatrixXXsc& C, VectorXs& c, VectorXs& xl, VectorXs& xu, VectorXs& beta, VectorXs& lambda )
{
  #ifdef QL_FOUND
  // QL only supports doubles
  assert( typeid(scalar) == typeid(double) );

  // All constraints are bound constraints.
  int m = 0;
  int me = 0;
  int mmax = 0;

  // C should be symmetric
  assert( C.rows() == C.cols() );
  assert( ( C - C.transpose() ).lpNorm<Eigen::Infinity>() < 1.0e-14 );
  // Number of degrees of freedom.
  int n = C.rows();
  int nmax = n;
  int mnn = m + n + n;

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
  assert( m == 0 && me == 0 && mmax == 0 );
  int lwar = 3 * ( nmax * nmax ) / 2 + 10 * nmax + 2;
  Eigen::VectorXd war( lwar );
  // Additional working space
  int liwar = n;
  Eigen::VectorXi iwar( liwar );

  {
    scalar local_tol = tol;
    ql_( &m, &me, &mmax,
         &n, &nmax, &mnn,
         C.data(), c.data(),
         nullptr, nullptr,
         xl.data(), xu.data(),
         beta.data(), u.data(),
         &local_tol,
         &mode, &iout, &ifail, &iprint,
         war.data(), &lwar,
         iwar.data(), &liwar );
  }

  assert( ( beta.array() <= xu.array() + tol ).all() ); assert( ( beta.array() >= xl.array() - tol ).all() );

  assert( u.size() % 2 == 0 );
  assert( u.segment( 0, u.size() / 2 ).dot( u.segment( u.size() / 2, u.size() / 2 ) ) == 0.0 );
  lambda = u.segment( 0, u.size() / 2 ).array().max( u.segment( u.size() / 2, u.size() / 2 ).array() );

  return ifail;
  #else
  std::cerr << " Error, please rebuild with QL support before executing BoundConstrainedMDPOperatorQL::solveQP." << std::endl;
  std::cerr << "            QL can be obtained via: http://www.ai7.uni-bayreuth.de/ql.htm" << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void BoundConstrainedMDPOperatorQL::flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda )
{
  // Quadratic term in the objective: 1/2 x^T C x
  assert( Q.rows() == Q.cols() );
  MatrixXXsc C{ Q };
  assert( ( C - C.transpose() ).lpNorm<Eigen::Infinity>() < 1.0e-6 );

  // Linear term in the objective: c^T x
  assert( D.rows() == v0.size() ); assert( D.cols() == gdotD.size() );
  VectorXs c{ D.transpose() * v0 + gdotD }; // No 2 as QL puts a 1/2 in front of the quadratic term

  // Bounds on variable values
  //if( ( alpha.array() < 0.0 ).any() ) std::cerr << "Negative alpha in BoundConstrainedMDPOperatorQL::flow: " << alpha.minCoeff() << std::endl;
  assert( ( mu.array() >= 0.0 ).all() ); assert( ( alpha.array() >= -1.0e-8 ).all() );
  VectorXs xl{ - mu.array() * alpha.array() };
  VectorXs xu{ - xl };

  const int status = solveQP( m_tol, C, c, xl, xu, beta, lambda );
  //std::cout << "beta: " << beta.transpose() << std::endl;
  //std::cout << "lambda: " << lambda.transpose() << std::endl;

  // Check for problems
  if( 0 != status )
  {
    std::cerr << "Warning, failed to solve QP in FrictionOperatorQL::flow: " << QLReturnStatusToString( status ) << "." << std::endl;
  }

  // TODO: Replace this check with a single call to a min-map functional
  #ifndef NDEBUG
  {
    // All impulses should be within the bound constraints (plus some wiggle room)
    assert( ( beta.array() >= xl.array() - 1.0e-7 ).all() );
    assert( ( beta.array() <= xu.array() + 1.0e-7 ).all() );
    // The cone multipliers should all be positive
    assert( lambda.all() >= 0.0 );
  }
  #endif
}

int BoundConstrainedMDPOperatorQL::numFrictionImpulsesPerNormal() const
{
  return 1;
}

void BoundConstrainedMDPOperatorQL::formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.rows() == v.size() ); assert( D.cols() == int( K.size() ) );
  FrictionOperatorUtilities::formGeneralizedFrictionBasis( q, v, K, 1, D, drel );
}

std::string BoundConstrainedMDPOperatorQL::name() const
{
  return "bound_constrained_mdp_ql";
}

std::unique_ptr<FrictionOperator> BoundConstrainedMDPOperatorQL::clone() const
{
  return std::unique_ptr<FrictionOperator>{ new BoundConstrainedMDPOperatorQL{ *this } };
}

void BoundConstrainedMDPOperatorQL::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeBuiltInType( m_tol, output_stream );
}

bool BoundConstrainedMDPOperatorQL::isLinearized() const
{
  return false;
}
