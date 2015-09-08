// MathUtilities.h
//
// Breannan Smith
// Last updated: 09/07/2015

#ifndef MATH_UTILITIES_H
#define MATH_UTILITIES_H

#include <Eigen/Core>
#include <Eigen/LU>
#include "MathDefines.h"

#include "SCISim/Utilities.h"

// TODO: Rename to MathUtilities
namespace mathutils
{

  // A 2D cross product
  inline scalar cross( const Vector2s& a, const Vector2s& b )
  {
    return a.x() * b.y() - a.y() * b.x();
  }

  unsigned computeNumDigits( unsigned n );

  VectorXs parallelTransport( const VectorXs& n0, const VectorXs& n1, const VectorXs& t0 );

  // Checks if the the two vectors when stacked as [ a b ] form an orthonormal matrix with positive determinant
  bool isRightHandedOrthoNormal( const Vector2s& a, const Vector2s& b, const scalar& tol );
  // Checks if the the three vectors when stacked as [ a b c ] form an orthonormal matrix with positive determinant
  bool isRightHandedOrthoNormal( const Vector3s& a, const Vector3s& b, const Vector3s& c, const scalar& tol );

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SPARSE MATRIX

  bool isSquare( const SparseMatrixsc& matrix );

  // Saves a sparse matrix in a triplet format that can be imported into Matlab via:
  //   load sparse_matrix.dat
  //   H = spconvert( sparse_matrix )
  bool writeToMatlabTripletText( const SparseMatrixsc& matrix, const std::string& file_name );

  // Converts a dense matrix to a sparse matrix, optionally omitting zero entries
  void convertDenseToSparse( const bool filter_zeros, const MatrixXXsc& dense_matrix, SparseMatrixsc& sparse_matrix );


  // The number of non-zeros on or below the diagonal
  int nzLowerTriangular( const SparseMatrixsc& A );

  // The number of non-zeros in A below the diagonal
  //int nzBelowDiagonal( const SparseMatrixsc& A );

  // Determine which elements are non-zero
  int sparsityPattern( const SparseMatrixsc& A, int* rows, int* cols );

  // Determine which elements on or below the diagonal are non-zero
  int sparsityPatternLowerTriangular( const SparseMatrixsc& A, int* rows, int* cols );

  // Extract elements
  int values( const SparseMatrixsc& A, scalar* vals );

  // Extract elements on or below the diagonal
  int valuesLowerTriangular( const SparseMatrixsc& A, scalar* vals );

  // Creates a sparse diagonal matrix D = c * Id
  void createDiagonalMatrix( const scalar& c, SparseMatrixsc& D );

  // Generates a Compressed Column Sparse array representation of a sparse matrix
  void extractDataCCS( const SparseMatrixsc& A, VectorXi& col_ptr, VectorXi& row_ind, VectorXs& val );

  // Generates a triplet representation of a sparse matrix
  void extractTripletData( const SparseMatrixsc& matrix, VectorXi& rows, VectorXi& cols, VectorXs& vals );

  // Extracts columns in cols from A0, in order, and places them in A1
  void extractColumns( const SparseMatrixsc& A0, const std::vector<unsigned>& cols, SparseMatrixsc& A1 );

  SparseMatrixsc sparseIdentity( const unsigned size );

  // TODO: These are ammenable to major speedups, if needed
  void extractLowerTriangularMatrix( const SparseMatrixsc& A, SparseMatrixsr& B );
  void extractLowerTriangularMatrix( const SparseMatrixsc& A, SparseMatrixsc& B );

  void printSparseMathematicaMatrix( const SparseMatrixsr& A, const scalar& eps );
  void printSparseMathematicaMatrix( const SparseMatrixsc& A, const scalar& eps );

  void serialize( const SparseMatrixsc& A, std::ostream& stm );
  void deserialize( SparseMatrixsc& A, std::istream& stm );

  template <typename ScalarType> inline
  void serialize( const Eigen::Quaternion<ScalarType>& a, std::ostream& stm )
  {
    assert( stm.good() );
    Utilities::serializeBuiltInType( a.x(), stm );
    Utilities::serializeBuiltInType( a.y(), stm );
    Utilities::serializeBuiltInType( a.z(), stm );
    Utilities::serializeBuiltInType( a.w(), stm );
  }

  template <typename ScalarType> inline
  void deserialize( Eigen::Quaternion<ScalarType>& a, std::istream& stm )
  {
    assert( stm.good() );
    a.x() = Utilities::deserialize<ScalarType>( stm );
    a.y() = Utilities::deserialize<ScalarType>( stm );
    a.z() = Utilities::deserialize<ScalarType>( stm );
    a.w() = Utilities::deserialize<ScalarType>( stm );
  }

  template <typename ScalarType> inline
  Eigen::Quaternion<ScalarType> deserializeQuaternion( std::istream& stm )
  {
    assert( stm.good() );
    Eigen::Quaternion<ScalarType> a;
    deserialize( a, stm );
    return a;
  }

  // TODO: Transition code to generic versions
  void serialize( const Vector2s& a, std::ostream& stm );
  void serialize( const Vector3s& a, std::ostream& stm );
  void serialize( const VectorXs& a, std::ostream& stm );
  void serialize( const Array3i& a, std::ostream& stm );
  void serialize( const Matrix3s& a, std::ostream& stm );
  void serialize( const Matrix3Xsc& a, std::ostream& stm );
  void serialize( const Matrix3Xuc& a, std::ostream& stm );
  void serialize( const Vector3u& a, std::ostream& stm );
  void serialize( const VectorXu& a, std::ostream& stm );

  // Deserialization for dense Eigen types of fixed row count, fixed column count
  template <typename Derived>
  typename std::enable_if< Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic, Derived >::type
   deserialize( std::istream& stm )
  {
    assert( stm.good() );
    Derived output_matrix;
    assert( output_matrix.rows() == Derived::RowsAtCompileTime );
    assert( output_matrix.cols() == Derived::ColsAtCompileTime );
    stm.read( reinterpret_cast<char*>( output_matrix.data() ), Derived::RowsAtCompileTime * Derived::ColsAtCompileTime * sizeof(typename Derived::Scalar) );
    assert( stm.good() );
    return output_matrix;
  }

  // Deserialization for dense Eigen types of dynamic row count, fixed column count
  template <typename Derived>
  typename std::enable_if< Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime == Eigen::Dynamic, Derived >::type
  deserialize( std::istream& stm )
  {
    assert( stm.good() );
    Derived output_matrix;
    assert( output_matrix.rows() == Derived::RowsAtCompileTime );
    {
      int ncols;
      stm.read( reinterpret_cast<char*>( &ncols ), sizeof(int) );
      output_matrix.resize( Derived::RowsAtCompileTime, ncols );
    }
    stm.read( reinterpret_cast<char*>( output_matrix.data() ), Derived::RowsAtCompileTime * output_matrix.cols() * sizeof(typename Derived::Scalar) );
    assert( stm.good() );
    return output_matrix;
  }

  // Deserialization for dense Eigen types of fixed row count, dynamic col count
  template <typename Derived>
  typename std::enable_if< Derived::RowsAtCompileTime == Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic, Derived >::type
  deserialize( std::istream& stm )
  {
    assert( stm.good() );
    Derived output_matrix;
    assert( output_matrix.cols() == Derived::ColsAtCompileTime );
    {
      int nrows;
      stm.read( reinterpret_cast<char*>( &nrows ), sizeof(int) );
      output_matrix.resize( nrows, Derived::ColsAtCompileTime );
    }
    stm.read( reinterpret_cast<char*>( output_matrix.data() ), output_matrix.rows() * Derived::ColsAtCompileTime * sizeof(typename Derived::Scalar) );
    assert( stm.good() );
    return output_matrix;
  }

  // TODO: Deserialization for dense Eigen types of dynamic row count, dynamic col count

}

#endif
