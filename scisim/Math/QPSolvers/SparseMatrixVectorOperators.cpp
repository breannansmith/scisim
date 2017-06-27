#include "SparseMatrixVectorOperators.h"

#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

#ifdef MKL_FOUND
#include "mkl.h"
#endif

scalar ObjectiveEigenColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x ) const
{
  assert( A.rows() == A.cols() );
  assert( A.rows() == b.size() );
  assert( A.rows() == x.size() );
  // NB: A.transpose() is faster, but requires A to be symmetric
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );
  return x.dot( 0.5 * A.transpose() * x + b );
}

void GradientEigenColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x, VectorXs& grad ) const
{
  assert( A.rows() == A.cols() );
  assert( A.rows() == b.size() );
  assert( A.rows() == x.size() );
  assert( A.rows() == grad.size() );
  // NB: A.transpose() is faster, but requires A to be symmetric
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );
  assert( grad.data() != x.data() );
  assert( grad.data() != b.data() );
  grad.noalias() = A.transpose() * x + b;
}

void MultiplyEigenColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& x, VectorXs& y ) const
{
  assert( A.rows() == A.cols() );
  assert( A.rows() == x.size() );
  assert( A.rows() == y.size() );
  // NB: A.transpose() is faster, but requires A to be symmetric
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );
  assert( y.data() != x.data() );
  y.noalias() = A.transpose() * x;
}

#ifdef MKL_FOUND
scalar ObjectiveMKLColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x ) const
{
  assert( A.rows() == A.cols() );
  assert( A.cols() == x.size() );
  assert( b.size() == x.size() );
  assert( typeid( SparseMatrixsc::Scalar ) == typeid( double ) );
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );

  char transa = 'n';
  int m = A.rows();
  int n = A.cols();
  char matdescra[] = "GxxCxx";
  scalar alpha = 0.5;
  scalar beta = 1.0;
  // y = alpha * A * x + beta * y
  // onehalfAxPlusb = 0.5 * A * x + b
  // TODO: cache this storage space?
  VectorXs one_half_A_x_plus_b{ b };
  mkl_dcsrmv( &transa, &m, &n, const_cast<double*>( &alpha ), matdescra, const_cast<double*>( A.valuePtr() ), const_cast<int*>( A.innerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() + 1 ), const_cast<double*>( x.data() ), &beta, one_half_A_x_plus_b.data() );
  return x.dot( one_half_A_x_plus_b );
}

void GradientMKLColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x, VectorXs& grad ) const
{
  assert( A.rows() == A.cols() );
  assert( A.cols() == x.size() );
  assert( b.size() == x.size() );
  assert( grad.size() == x.size() );
  assert( typeid( SparseMatrixsc::Scalar ) == typeid( double ) );
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );

  char transa = 'n';
  int m = A.rows();
  int n = A.cols();
  char matdescra[] = "GxxCxx";
  scalar alpha = 1.0;
  scalar beta = 1.0;
  grad = b;
  // y = alpha * A * x + beta * y
  // grad = A * x + b
  mkl_dcscmv( &transa, &m, &n, const_cast<double*>( &alpha ), matdescra, const_cast<double*>( A.valuePtr() ), const_cast<int*>( A.innerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() + 1 ), const_cast<double*>( x.data() ), &beta, grad.data() );
}

void MultiplyMKLColumnMajor::operator()( const SparseMatrixsc& A, const VectorXs& x, VectorXs& y ) const
{
  assert( A.rows() == A.cols() );
  assert( A.cols() == x.size() );
  assert( y.size() == x.size() );
  assert( typeid( SparseMatrixsc::Scalar ) == typeid( double ) );
  assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );

  char transa = 'n';
  int m = A.rows();
  int n = A.cols();
  char matdescra[] = "GxxCxx";
  scalar alpha = 1.0;
  scalar beta = 0.0;
  // TODO: Is setZero needed if beta == 0.0?
  y.setZero();
  // y = alpha * A * x + beta * y
  // y = A * x
  mkl_dcscmv( &transa, &m, &n, const_cast<double*>( &alpha ), matdescra, const_cast<double*>( A.valuePtr() ), const_cast<int*>( A.innerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() ), const_cast<int*>( A.outerIndexPtr() + 1 ), const_cast<double*>( x.data() ), &beta, y.data() );
}
#endif
