#ifndef SPARSE_MATRIX_VECTOR_OPERATORS_H
#define SPARSE_MATRIX_VECTOR_OPERATORS_H

#include "scisim/Math/MathDefines.h"

struct ObjectiveEigenColumnMajor final
{
  scalar operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};

struct GradientEigenColumnMajor final
{
  void operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x, VectorXs& grad ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};

struct MultiplyEigenColumnMajor final
{
  void operator()( const SparseMatrixsc& A, const VectorXs& x, VectorXs& y ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};

#ifdef MKL_FOUND
struct ObjectiveMKLColumnMajor final
{
  scalar operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};

struct GradientMKLColumnMajor final
{
  void operator()( const SparseMatrixsc& A, const VectorXs& b, const VectorXs& x, VectorXs& grad ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};

struct MultiplyMKLColumnMajor final
{
  void operator()( const SparseMatrixsc& A, const VectorXs& x, VectorXs& y ) const;

  static constexpr bool columnMajor()
  {
    return true;
  }
};
#endif

#endif
