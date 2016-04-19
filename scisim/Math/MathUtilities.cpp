// MathUtilities.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

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

void MathUtilities::extractDataCCS( const SparseMatrixsc& A, VectorXi& col_ptr, VectorXi& row_ind, VectorXs& val )
{
  col_ptr.resize( A.cols() + 1 );
  row_ind.resize( A.nonZeros() );
  val.resize( A.nonZeros() );

  col_ptr(0) = 0;
  for( int col = 0; col < A.outerSize(); ++col )
  {
    col_ptr(col+1) = col_ptr(col);
    for( SparseMatrixsc::InnerIterator it(A,col); it; ++it )
    {
      const int row{ it.row() };

      val(col_ptr(col+1)) = it.value();
      row_ind(col_ptr(col+1)) = row;
      ++col_ptr(col+1);
    }
  }

  assert( col_ptr( col_ptr.size() - 1 ) == row_ind.size() );
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

  VectorXi col_ptr;
  VectorXi row_ind;
  VectorXs val;
  MathUtilities::extractDataCCS( A, col_ptr, row_ind, val );
  assert( col_ptr.size() == A.cols() + 1 ); assert( row_ind.size() == A.nonZeros() ); assert( val.size() == A.nonZeros() );
  // Size of col_ptr == A.cols() + 1
  Utilities::serializeBuiltInType( A.rows(), stm );
  Utilities::serializeBuiltInType( A.cols(), stm );
  stm.write( reinterpret_cast<char*>( col_ptr.data() ), col_ptr.size() * sizeof(int) );
  // Size of row_ind == size of val == A.nonZeros()
  Utilities::serializeBuiltInType( A.nonZeros(), stm );
  stm.write( reinterpret_cast<char*>( row_ind.data() ), row_ind.size() * sizeof(int) );
  stm.write( reinterpret_cast<char*>( val.data() ), val.size() * sizeof(scalar) );
}

// TODO: Use utility class here
void MathUtilities::deserialize( SparseMatrixsc& A, std::istream& stm )
{
  assert( stm.good() );

  VectorXi col_ptr;
  VectorXi row_ind;
  VectorXs val;

  // Read the number of rows in the matrix
  int rows{ -1 };
  stm.read((char*)&rows,sizeof(int));
  assert( rows >= 0 );
  // Read the number of columns in the matrix
  int cols{ -1 };
  stm.read((char*)&cols,sizeof(int));
  assert( cols >= 0 );
  // Read the column pointer array
  col_ptr.resize(cols+1);
  stm.read((char*)col_ptr.data(),col_ptr.size()*sizeof(int));
  // Read the number of nonzeros in the array
  int nnz{ -1 };
  stm.read((char*)&nnz,sizeof(int));
  assert( nnz >= 0 );
  // Read the row-index array
  row_ind.resize(nnz);
  stm.read((char*)row_ind.data(),row_ind.size()*sizeof(int));
  // Read the value array
  val.resize(nnz);
  stm.read((char*)val.data(),val.size()*sizeof(scalar));

  A.resize(rows,cols);
  A.reserve(nnz);

  for( int col = 0; col < cols; ++col )
  {
    A.startVec(col);
    for( int curel = col_ptr(col); curel < col_ptr(col+1); ++curel )
    {
      int row = row_ind(curel);
      scalar curval = val(curel);
      A.insertBack(row,col) = curval;
    }
  }
  A.finalize();
}
