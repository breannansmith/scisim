// LCPOperatorQL.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "LCPOperatorQL.h"

#include "ImpactOperatorUtilities.h"
#include "scisim/Utilities.h"
#include "scisim/Math/QL/QLUtilities.h"

#include <iostream>

#ifndef NDEBUG
#ifdef QL_FOUND
#include <typeinfo>
#endif
#endif

LCPOperatorQL::LCPOperatorQL( const scalar& eps )
: m_tol( eps )
{
  assert( m_tol >= 0.0 );
}

LCPOperatorQL::LCPOperatorQL( std::istream& input_stream )
: m_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_tol >= 0.0 );
}

LCPOperatorQL::LCPOperatorQL( const LCPOperatorQL& other )
: m_tol( other.m_tol )
{
  assert( m_tol >= 0.0 );
}

static int solveQP( const scalar& tol, MatrixXXsc& C, VectorXs& dvec, VectorXs& alpha )
{
  #ifdef QL_FOUND
  // QL only supports doubles
  assert( typeid(scalar).name() == typeid(double).name() );
  assert( dvec.size() == alpha.size() );

  // All constraints are bound constraints.
  int m = 0;
  int me = 0;
  int mmax = 0;

  // C should be symmetric
  assert( ( C - C.transpose() ).lpNorm<Eigen::Infinity>() < 1.0e-14 );
  // Number of degrees of freedom.
  assert( C.rows() == C.cols() );
  int n = C.rows();
  int nmax = n;
  int mnn = m + n + n;
  assert( dvec.size() == nmax );

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
  assert( m == 0 && me == 0 && mmax == 0 );
  int lwar = 3 * ( nmax * nmax ) / 2 + 10 * nmax + 2;
  Eigen::VectorXd war( lwar );
  // Additional working space
  int liwar = n;
  Eigen::VectorXi iwar( liwar );

  {
    scalar tol_local = tol;
    ql_( &m,
        &me,
        &mmax,
        &n,
        &nmax,
        &mnn,
        C.data(),
        dvec.data(),
        nullptr,
        nullptr,
        xl.data(),
        xu.data(),
        alpha.data(),
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

  return ifail;
  #else
  std::cerr << " Error, please rebuild with QL support before executing LCPOperatorQL::solveQP." << std::endl;
  std::cerr << "            QL can be obtained via: http://www.ai7.uni-bayreuth.de/ql.htm" << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void LCPOperatorQL::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  // Q in 1/2 \alpha^T Q \alpha
  assert( Q.rows() == Q.cols() );
  MatrixXXsc Qdense = Q;

  // Linear term in the objective
  VectorXs Adense;
  ImpactOperatorUtilities::computeLCPQPLinearTerm( N, nrel, CoR, v0, v0F, Adense );

  // Solve the QP
  assert( Qdense.rows() == Adense.size() ); assert( Adense.size() == alpha.size() );
  const int status = solveQP( m_tol, Qdense, Adense, alpha );

  // Check for problems
  if( 0 != status )
  {
    std::cerr << "Warning, failed to solve QP in LCPOperatorQL::flow: " << QLUtilities::QLReturnStatusToString(status) << std::endl;
  }

  // TODO: Sanity check the solution here
}

std::string LCPOperatorQL::name() const
{
  return "lcp_ql";
}

std::unique_ptr<ImpactOperator> LCPOperatorQL::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new LCPOperatorQL{ *this } };
}

void LCPOperatorQL::serialize( std::ostream& output_stream ) const
{
  Utilities::serializeBuiltInType( m_tol, output_stream );
}
