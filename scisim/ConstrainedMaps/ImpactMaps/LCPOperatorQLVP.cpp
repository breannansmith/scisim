// LCPOperatorQLVP.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "LCPOperatorQLVP.h"

//#include "ImpactOperatorUtilities.h"

#include <iostream>

LCPOperatorQLVP::LCPOperatorQLVP( const scalar& tol )
: m_tol( tol )
{
  assert( m_tol >= 0.0 );
}

LCPOperatorQLVP::LCPOperatorQLVP( const LCPOperatorQLVP& other )
: m_tol( other.m_tol )
{
  assert( m_tol >= 0.0 );
}

//static std::string QLReturnStatusToString( const int status )
//{
//  if( 0 == status )
//  {
//    return "The optimality conditions are satisfied";
//  }
//  else if( 1 == status )
//  {
//    return "The algorithm has been stopped after too many MAXIT iterations (40*(N+M)";
//  }
//  else if( 2 == status )
//  {
//    return "Termination accuracy insufficient to satisfy convergence criterion";
//  }
//  else if( 3 == status )
//  {
//    return "Internal inconsistency of QL, division by zero";
//  }
//  else if( 5 == status )
//  {
//    return "Length of a working array is too short";
//  }
//  else if( 100 < status )
//  {
//    return "Constraints are inconsistent and IFAIL=100+ICON, where ICON denotes a constraint causing the conflict: " + StringUtilities::convertToString( status );
//  }
//
//  std::cerr << "Unhandled error message in LCPOperatorQL::QLReturnStatusToString. Probably a bug." << std::endl;
//  std::exit(EXIT_FAILURE);
//}

//static int solveQP( const scalar& tol, MatrixXXsc& C, VectorXs& c, MatrixXXsc& A, VectorXs& b, VectorXs& x, VectorXs& lambda )
//{
//  #ifdef QL_FOUND
//  assert( typeid(scalar) == typeid(double) );  // QL only supports doubles
//  assert( C.rows() == C.cols() ); assert( C.rows() == c.size() ); assert( C.rows() == x.size() );
//  assert( C.rows() == A.cols() ); assert( A.rows() == b.size() ); assert( b.size() == lambda.size() );
//
//  // Inequality constraints
//  int m = b.size();
//  // No equality constraints
//  int me = 0;
//  // Row size of matrix containing linear constraints
//  int mmax = A.rows();
//
//  // Number of degrees of freedom.
//  int n = C.rows();
//  int nmax = n;
//  int mnn = m+n+n;
//
//  // No constraints on the variables
//  Eigen::VectorXd xl = Eigen::VectorXd::Constant( nmax, -std::numeric_limits<double>::infinity() );
//  Eigen::VectorXd xu = Eigen::VectorXd::Constant( nmax, std::numeric_limits<double>::infinity() );
//
//  // u will contain the constraint multipliers
//  Eigen::VectorXd u( mnn );
//
//  // Use the built-in cholesky decomposition
//  int mode = 1;
//  // Some fortran output stuff
//  int iout = 0;
//  // 1 => print output, 0 => silent
//  int iprint = 1;
//
//  // Status of the solve
//  int ifail = -1;
//  // Working space
//  int lwar = 3 * nmax * nmax / 2 + 10 * nmax + mmax + m + 2;
//  Eigen::VectorXd war( lwar );
//  // Additional working space
//  int liwar = n;
//  Eigen::VectorXi iwar( liwar );
//
//  {
//    scalar tol_local = tol;
//    ql_( &m,
//        &me,
//        &mmax,
//        &n,
//        &nmax,
//        &mnn,
//        C.data(),
//        c.data(),
//        A.data(),
//        b.data(),
//        xl.data(),
//        xu.data(),
//        x.data(),
//        u.data(),
//        &tol_local,
//        &mode,
//        &iout,
//        &ifail,
//        &iprint,
//        war.data(),
//        &lwar,
//        iwar.data(),
//        &liwar );
//  }
//
//  // Save out the inequality constraint multipliers
//  lambda = u.segment( 0, lambda.size() );
//
//  return ifail;
//  #else
//  std::cerr << " Error, please rebuild with QL support before executing LCPOperatorQLVP::solveQP." << std::endl;
//  std::cerr << "            QL can be obtained via: http://www.ai7.uni-bayreuth.de/ql.htm" << std::endl;
//  std::exit(EXIT_FAILURE);
//  #endif
//}

void LCPOperatorQLVP::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  std::cerr << "Error, LCPOperatorQLVP::flow not updated to work with new interface" << std::endl;
  std::exit( EXIT_FAILURE );
////  assert( lambda.size() == (int) K.size() );
//
//  // C in objective: 1/2 x^T C x + c^T x
//  // C = M
//  MatrixXXsc C = M;
//  assert( C.rows() == C.cols() ); assert( C.rows() == v0.size() );
//
//  // c in 1/2 x^T C x + c^T x
//  // c = - M v^-
//  VectorXs c = - M * v0F;
//
//  // A in A^T x + b >= 0
//  // A = N
//  MatrixXXsc A = N.transpose();
//
//  // b in A^T x + b >= 0
//  // b = c_r N^T v^-
//  assert( CoR >= 0.0 ); assert( CoR <= 1.0 );
//  // TODO: nrel goes here
//  VectorXs b = CoR * N.transpose() * v0;
//
//  // TODO: Call the correct solveQP function
//  const int status = 1; //solveQP( C, c, A, b, v1, lambda );
//
//  // Check for problems
//  if( 0 != status )
//  {
//    std::cout << "Warning, failed to solve QP in LCPOperatorQLVP::flow: " << QLReturnStatusToString(status) << std::endl;
//  }
//
//  // Impulses should be non-negative
//  // TODO: Check alpha
//  //assert( ( lambda.array() >= 0.0 ).all() );
//  // Velocities should satisfy linear constraints
//  // TODO: Check constraints
//  //assert( ( ( A * v1 + b ).array() >= -1.0e-6 ).all() );
//
////  // Impulses should give the outgoing velocity 
////  #ifndef NDEBUG
////  {
////    const VectorXs vtest = v0F + fsys.Minv() * A.transpose() * lambda;
////    assert( ( v1 - vtest ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
////  }
////  #endif
}

std::string LCPOperatorQLVP::name() const
{
  return "lcp_ql_vp";
}

std::unique_ptr<ImpactOperator> LCPOperatorQLVP::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new LCPOperatorQLVP{ *this } };
}

void LCPOperatorQLVP::serialize( std::ostream& output_stream ) const
{
  std::cerr << "Code up LCPOperatorQLVP::serialize" << std::endl;
  std::exit( EXIT_FAILURE );
}
