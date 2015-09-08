// MathUtilities.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "MathUtilities.h"

#include <iostream>
#include <fstream>

static VectorXs parallelTransport2D( const VectorXs& n0, const VectorXs& n1, const VectorXs& t0 )
{
  assert( fabs( n0.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( n1.norm() - 1.0 ) <= 1.0e-6 );

  // x is cos of angle, y is sin of angle
  const Vector2s r{ n0.dot( n1 ), mathutils::cross( n0, n1 ) };
  assert( fabs( r.norm() - 1.0 ) <= 1.0e-6 );

  // Rotate t0
  VectorXs t1{ 2 };
  t1( 0 ) = r.x() * t0.x() - r.y() * t0.y();
  t1( 1 ) = r.y() * t0.x() + r.x() * t0.y();
  assert( fabs( t0.norm() - t1.norm() ) <= 1.0e-6 );
  // TODO: Assert n0,t0 angle is same as n1,t1 angle

  return t1;
}

static VectorXs parallelTransport3D( const VectorXs& n0, const VectorXs& n1, const VectorXs& t0 )
{
  assert( false );
  return VectorXs::Constant( 3, SCALAR_NAN );
}

VectorXs mathutils::parallelTransport( const VectorXs& n0, const VectorXs& n1, const VectorXs& t0 )
{
  assert( n0.size() == n1.size() ); assert( n0.size() == n1.size() );
  assert( n0.size() == 2 || n0.size() == 3 );
  if( n0.size() == 2 ) { return parallelTransport2D( n0, n1, t0 ); }
  if( n0.size() == 3 ) { return parallelTransport3D( n0, n1, t0 ); }
  std::cerr << "Impossible code path in parallel transport, exiting." << std::endl;
  std::exit( EXIT_FAILURE );
}

void mathutils::extractTripletData( const SparseMatrixsc& matrix, VectorXi& rows, VectorXi& cols, VectorXs& vals )
{
  rows.resize( matrix.nonZeros() );
  cols.resize( matrix.nonZeros() );
  vals.resize( matrix.nonZeros() );
  int flat_index{ 0 };
  for( int outer_index = 0; outer_index < matrix.outerSize(); ++outer_index )
  {
    for( Eigen::SparseMatrix<double>::InnerIterator it( matrix, outer_index ); it; ++it )
    {
      rows( flat_index ) = it.row();
      cols( flat_index ) = it.col();
      vals( flat_index++ ) = it.value();
    }
  }
  assert( flat_index == matrix.nonZeros() );
}

bool mathutils::isRightHandedOrthoNormal( const Vector2s& a, const Vector2s& b, const scalar& tol )
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

bool mathutils::isRightHandedOrthoNormal( const Vector3s& a, const Vector3s& b, const Vector3s& c, const scalar& tol )
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

bool mathutils::isSquare( const SparseMatrixsc& matrix )
{
  return matrix.rows() == matrix.cols();
}

bool mathutils::writeToMatlabTripletText( const SparseMatrixsc& matrix, const std::string& file_name )
{
  std::ofstream output_file( file_name );
  if( !output_file.is_open() )
  {
    return false;
  }
  for( int outer_idx = 0; outer_idx < matrix.outerSize(); ++outer_idx )
  {
    for( SparseMatrixsc::InnerIterator it( matrix, outer_idx ); it; ++it )
    {
      // Matlab is 1 indexed
      output_file << it.row() + 1 << "\t" << it.col() + 1 << "\t" << it.value() << std::endl;
    }
  }
  return true;
}

void mathutils::convertDenseToSparse( const bool filter_zeros, const MatrixXXsc& dense_matrix, SparseMatrixsc& sparse_matrix )
{
  std::vector<Eigen::Triplet<scalar>> triplets;
  for( int row = 0; row < dense_matrix.rows(); ++row )
  {
    for( int col = 0; col < dense_matrix.cols(); ++col )
    {
      if( dense_matrix( row, col ) != 0.0 || !filter_zeros )
      {
        triplets.emplace_back( Eigen::Triplet<scalar>{ row, col, dense_matrix( row, col ) } );
      }
    }
  }
  sparse_matrix.resize( dense_matrix.rows(), dense_matrix.cols() );
  sparse_matrix.setFromTriplets( std::begin( triplets ), std::end( triplets ) );
  sparse_matrix.makeCompressed();
}

SparseMatrixsc mathutils::sparseIdentity( const unsigned size )
{
  SparseMatrixsc id_mat( size, size );
  id_mat.setIdentity();
  return id_mat;
}



unsigned mathutils::computeNumDigits( unsigned n )
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

int mathutils::nzLowerTriangular( const SparseMatrixsc& A )
{
  int num{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      // Skip entries above the diagonal
      if( col > it.row() ) { continue; }
      ++num;
    }
  }
  return num;
}

// Determine which elements are non-zero
int mathutils::sparsityPattern( const SparseMatrixsc& A, int* rows, int* cols )
{
  assert( rows != nullptr );
  assert( cols != nullptr );
  
  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      rows[curel] = it.row();
      cols[curel] = col;
      ++curel;
    }
  }

  assert( curel == A.nonZeros() );
  return curel;
}

int mathutils::sparsityPatternLowerTriangular( const SparseMatrixsc& A, int* rows, int* cols )
{
  assert( rows != nullptr );
  assert( cols != nullptr );

  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() ) continue;
      rows[curel] = it.row();
      cols[curel] = col;
      ++curel;
    }
  }

  return curel;
}

int mathutils::values( const SparseMatrixsc& A, scalar* vals )
{
  assert( vals != nullptr );
  
  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      vals[curel] = it.value();
      ++curel;
    }
  }
  
  assert( curel == A.nonZeros() );
  return curel;
}

int mathutils::valuesLowerTriangular( const SparseMatrixsc& A, scalar* vals )
{
  assert( vals != nullptr );

  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() ) continue;
      vals[curel] = it.value();
      ++curel;
    }
  }

  return curel;
}

void mathutils::createDiagonalMatrix( const scalar& c, SparseMatrixsc& D )
{
  assert( D.rows() == D.cols() );
  D.reserve( VectorXi::Constant( D.cols(), 1 ) );
  for( int i = 0; i < D.cols(); ++i ) { D.insert(i,i) = c; }
  D.makeCompressed();
}

void mathutils::extractDataCCS( const SparseMatrixsc& A, VectorXi& col_ptr, VectorXi& row_ind, VectorXs& val )
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
void mathutils::extractColumns( const SparseMatrixsc& A0, const std::vector<unsigned>& cols, SparseMatrixsc& A1 )
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

void mathutils::extractLowerTriangularMatrix( const SparseMatrixsc& A, SparseMatrixsr& B )
{
  std::vector< Eigen::Triplet<scalar> > triplets;
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() ) { continue; }
      triplets.push_back( Eigen::Triplet<scalar>( it.row(), col, it.value() ) );
    }
  }
  B.resize( A.rows(), A.cols() );
  B.setFromTriplets( triplets.begin(), triplets.end() );
  B.makeCompressed();
}
  
void mathutils::extractLowerTriangularMatrix( const SparseMatrixsc& A, SparseMatrixsc& B )
{
  std::vector< Eigen::Triplet<scalar> > triplets;
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() ) { continue; }
      triplets.push_back( Eigen::Triplet<scalar>( it.row(), col, it.value() ) );
    }
  }
  B.resize( A.rows(), A.cols() );
  B.setFromTriplets( triplets.begin(), triplets.end() );
  B.makeCompressed();
}
  
void mathutils::printSparseMathematicaMatrix( const SparseMatrixsr& A, const scalar& eps )
{
  std::cout << "{";
  int entry_num{ 0 };
  for( int k = 0; k < A.outerSize(); ++k )
  {
    for( typename SparseMatrixsr::InnerIterator it(A,k); it; ++it )
    {
      std::cout << "{" << (it.row()+1) << "," << (it.col()+1) << "}->";
      if( fabs(it.value()) < eps ) { std::cout << 0.0; }
      else { std::cout << it.value(); }
      entry_num++;
      if( entry_num != A.nonZeros() ) { std::cout << ","; }
    }
  }
  std::cout << "}" << std::endl;
}
  
void mathutils::printSparseMathematicaMatrix( const SparseMatrixsc& A, const scalar& eps )
{
  std::cout << "{";
  int entry_num = 0;
  for( int k = 0; k < A.outerSize(); ++k )
  {
    for( typename SparseMatrixsc::InnerIterator it(A,k); it; ++it )
    {
      std::cout << "{" << (it.row()+1) << "," << (it.col()+1) << "}->";
      if( fabs(it.value()) < eps ) { std::cout << 0.0; }
      else { std::cout << it.value(); }
      entry_num++;
      if( entry_num != A.nonZeros() ) { std::cout << ","; }
    }
  }
  std::cout << "}" << std::endl;
}

void mathutils::serialize( const SparseMatrixsc& A, std::ostream& stm )
{
  assert( stm.good() );

  VectorXi col_ptr;
  VectorXi row_ind;
  VectorXs val;
  mathutils::extractDataCCS( A, col_ptr, row_ind, val );
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
void mathutils::deserialize( SparseMatrixsc& A, std::istream& stm )
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

void mathutils::serialize( const Vector2s& a, std::ostream& stm )
{
  assert( stm.good() );
  stm.write( (char*) a.data(), a.size() * sizeof(scalar) );
}

void mathutils::serialize( const Vector3s& a, std::ostream& stm )
{
  assert( stm.good() );
  stm.write( (char*) a.data(), a.size() * sizeof(scalar) );
}

void mathutils::serialize( const VectorXs& a, std::ostream& stm )
{
  assert( stm.good() );
  const int asize = a.size();
  stm.write( (char*) &asize, sizeof(int) );
  stm.write( (char*) a.data(), a.size() * sizeof(scalar) );
}

void mathutils::serialize( const VectorXu& a, std::ostream& stm )
{
  assert( stm.good() );
  int asize = a.size();
  stm.write( reinterpret_cast<char*>( &asize ), sizeof(int) );
  stm.write( const_cast<char*>( reinterpret_cast<const char*>( a.data() ) ), a.size() * sizeof(unsigned) );
}

void mathutils::serialize( const Array3i& a, std::ostream& stm )
{
  assert( stm.good() );
  stm.write( (char*) a.data(), a.size() * sizeof(int) );
}

void mathutils::serialize( const Matrix3s& a, std::ostream& stm )
{
  assert( stm.good() );
  stm.write( (char*) a.data(), a.rows() * a.cols() * sizeof(scalar) );
}

void mathutils::serialize( const Matrix3Xsc& a, std::ostream& stm )
{
  assert( stm.good() );
  {
    int ncols = a.cols();
    stm.write( reinterpret_cast<char*>( &ncols ), sizeof(int) );
  }
  stm.write( (char*) a.data(), a.rows() * a.cols() * sizeof(scalar) );
}

void mathutils::serialize( const Matrix3Xuc& a, std::ostream& stm )
{
  assert( stm.good() );
  {
    int ncols = a.cols();
    stm.write( reinterpret_cast<char*>( &ncols ), sizeof(int) );
  }
  stm.write( (char*) a.data(), a.rows() * a.cols() * sizeof(unsigned) );
}

void mathutils::serialize( const Vector3u& a, std::ostream& stm )
{
  assert( stm.good() );
  stm.write( (char*) a.data(), a.rows() * a.cols() * sizeof(unsigned) );
}
