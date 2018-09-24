#include "MathUtilities.h"

#include <fstream>

bool MathUtilities::isRightHandedOrthoNormal( const Vector2s& a, const Vector2s& b, const scalar& tol )
{
  // All basis vectors should be unit
  if( fabs( a.norm() - 1.0 ) > tol ) { return false; }
  if( fabs( b.norm() - 1.0 ) > tol ) { return false; }
  // All basis vectors should be mutually orthogonal
  if( fabs( a.dot( b ) ) > tol ) { return false; }
  // Coordinate system should be right handed
  assert( fabs( cross( a, b ) - 1.0 ) <= 1.0e-6 || fabs( cross( a, b ) + 1.0 ) <= 1.0e-6 );
  if( cross( a, b ) <= 0.0 ) { return false; }
  return true;
}

bool MathUtilities::isRightHandedOrthoNormal( const Vector3s& a, const Vector3s& b, const Vector3s& c, const scalar& tol )
{
  // All basis vectors should be unit
  if( fabs( a.norm() - 1.0 ) > tol ) { return false; }
  if( fabs( b.norm() - 1.0 ) > tol ) { return false; }
  if( fabs( c.norm() - 1.0 ) > tol ) { return false; }
  // All basis vectors should be mutually orthogonal
  if( fabs( a.dot( b ) ) > tol ) { return false; }
  if( fabs( a.dot( c ) ) > tol ) { return false; }
  if( fabs( b.dot( c ) ) > tol ) { return false; }
  // Coordinate system should be right handed
  if( ( a.cross( b ) - c ).lpNorm<Eigen::Infinity>() > tol ) { return false; }
  return true;
}

bool MathUtilities::isSquare( const SparseMatrixsc& matrix )
{
  return matrix.rows() == matrix.cols();
}

bool MathUtilities::isIdentity( const SparseMatrixsc& A, const scalar& tol )
{
  if( !isSquare( A ) )
  {
    return false;
  }
  for( int outer_idx = 0; outer_idx < A.outerSize(); ++outer_idx )
  {
    for( SparseMatrixsc::InnerIterator it( A, outer_idx ); it; ++it )
    {
      if( it.row() == it.col() )
      {
        if( fabs( it.value() - 1.0 ) > tol )
        {
          return false;
        }
      }
      else
      {
        if( fabs( it.value() ) > tol )
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool MathUtilities::isSymmetric( const SparseMatrixsc& A, const scalar& tol )
{
  const SparseMatrixsc& B{ A - SparseMatrixsc{ A.transpose() } };
  for( int outer_idx = 0; outer_idx < B.outerSize(); ++outer_idx )
  {
    for( SparseMatrixsc::InnerIterator it( B, outer_idx ); it; ++it )
    {
      if( fabs( it.value() ) > tol )
      {
        return false;
      }
    }
  }
  return true;
}

unsigned MathUtilities::computeNumDigits( unsigned n )
{
  if( n == 0 ) { return 1; }
  unsigned num_digits{ 0 };
  while( n != 0 )
  {
    n /= 10;
    ++num_digits;
  }
  return num_digits;
}

// TODO: Pull the outerIndexPtr arithmetic into a helper function
void MathUtilities::extractColumns( const SparseMatrixsc& A0, const std::vector<unsigned>& cols, SparseMatrixsc& A1 )
{
  const unsigned ncols_to_extract{ static_cast<unsigned>( cols.size() ) };

  assert( ncols_to_extract <= static_cast<unsigned>( A0.cols() ) );
  #ifndef NDEBUG
  for( unsigned i = 0; i < ncols_to_extract; ++i )
  {
    assert( cols[i] < unsigned( A0.cols() ) );
  }
  #endif
    
  // Compute the number of nonzeros in each column of the new matrix
  VectorXi column_nonzeros{ ncols_to_extract };
  for( unsigned i = 0; i < ncols_to_extract; ++i )
  {
    column_nonzeros( i ) = A0.outerIndexPtr()[cols[i]+1] - A0.outerIndexPtr()[cols[i]];
  }

  // Resize A1 and reserve space
  A1.resize( A0.rows(), ncols_to_extract );
  A1.reserve( column_nonzeros );
  // Copy the data over, column by column
  for( unsigned cur_col = 0; cur_col < ncols_to_extract; ++cur_col )
  {
    for( SparseMatrixsc::InnerIterator it( A0, cols[ cur_col ] ); it; ++it )
    {
      A1.insert( it.row(), cur_col ) = it.value();
    }
  }
  
  A1.makeCompressed();
  
  #ifndef NDEBUG
  for( int i = 0 ; i < A1.cols(); ++i )
  {
    assert( ( A1.outerIndexPtr()[i+1] - A1.outerIndexPtr()[i] ) == column_nonzeros( i ) );
  }
  #endif
}

void MathUtilities::serialize( const SparseMatrixsc& A, std::ostream& stm )
{
  assert( stm.good() );
  assert( A.isCompressed() );

  Utilities::serialize( A.rows(), stm );
  Utilities::serialize( A.cols(), stm );
  Utilities::serialize( A.nonZeros(), stm );
  // TODO: Write a utility class to save out pointers to arrays of data
  stm.write( reinterpret_cast<const char*>( A.innerIndexPtr() ), A.nonZeros() * sizeof(SparseMatrixsc::StorageIndex) );
  stm.write( reinterpret_cast<const char*>( A.outerIndexPtr() ), ( A.outerSize() + 1 ) * sizeof(SparseMatrixsc::StorageIndex) );
  stm.write( reinterpret_cast<const char*>( A.valuePtr() ), A.nonZeros() * sizeof(SparseMatrixsc::Scalar) );
}

void MathUtilities::deserialize( SparseMatrixsc& A, std::istream& stm )
{
  assert( stm.good() );

  const Eigen::Index rows{ Utilities::deserialize<Eigen::Index>( stm ) };
  const Eigen::Index cols{ Utilities::deserialize<Eigen::Index>( stm ) };
  const Eigen::Index nnz{ Utilities::deserialize<Eigen::Index>( stm ) };

  Eigen::Matrix<SparseMatrixsc::StorageIndex,Eigen::Dynamic,1> inner_indices{ nnz };
  stm.read( reinterpret_cast<char*>( inner_indices.data() ), inner_indices.size() * sizeof(SparseMatrixsc::StorageIndex) );

  Eigen::Matrix<SparseMatrixsc::StorageIndex,Eigen::Dynamic,1> outter_indices{ cols + 1 };
  stm.read( reinterpret_cast<char*>( outter_indices.data() ), outter_indices.size() * sizeof(SparseMatrixsc::StorageIndex) );

  Eigen::Matrix<SparseMatrixsc::Scalar,Eigen::Dynamic,1> values{ nnz };
  stm.read( reinterpret_cast<char*>( values.data() ), values.size() * sizeof(SparseMatrixsc::Scalar) );

  const Eigen::Map<const SparseMatrixsc> matrix_map( rows, cols, nnz, outter_indices.data(), inner_indices.data(), values.data() );

  A = matrix_map;
  assert( A.isCompressed() );
}
