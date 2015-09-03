// SparseMatrixUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "SparseMatrixUtilities.h"

static void nodeDegree( const Eigen::Map<const VectorXi>& ind, const Eigen::Map<const VectorXi>& ptr, VectorXi& degree )
{
  assert( ind.size() == ptr.size() - 1 );

  for( int i = 0; i < degree.size(); ++i )
  {
    degree(i) = ptr(i+1) - ptr(i);
    for( int j = ptr(i); j < ptr(i+1); ++j )
    {
      if( ind(j) == i )
      {
        degree(i) += 1;
        break;
      }
    }
  }
}

static void argsort( const VectorXi& numbers, VectorXi& indexes )
{
  assert( indexes.size() == numbers.size() );
  for( int i = 0; i < indexes.size(); ++i ) indexes[i] = i;
  std::sort( indexes.data(), indexes.data() + indexes.size(), [&numbers]( size_t i1, size_t i2 ) { return numbers(i1) < numbers(i2); } );
}

static void cuthillMckeee( const Eigen::Map<const VectorXi>& ind, const Eigen::Map<const VectorXi>& ptr, VectorXi& permutation )
{
  VectorXi degree( permutation.size() );
  nodeDegree( ind, ptr, degree );
  VectorXi inds( degree.size() );
  argsort( degree, inds );
  VectorXi rev_inds( inds.size() );
  argsort( inds, rev_inds );
  VectorXu tmp_degrees = VectorXu::Zero( permutation.size() );

  unsigned N = 0;
  for( unsigned z = 0; z < permutation.size(); ++z )
  {
    if( inds(z) != -1 )
    {
      const unsigned seed = inds(z);
      permutation(N) = seed;
      N += 1;
      inds(rev_inds(seed)) = -1;
      unsigned level_start = N - 1;
      unsigned level_end = N;

      while( level_start < level_end )
      {
        for( unsigned ii = level_start; ii < level_end; ++ii )
        {
          const unsigned i = permutation(ii);
          const unsigned N_old = N;

          for( int jj = ptr(i); jj < ptr(i+1); ++jj )
          {
            const unsigned j = ind(jj);
            if( inds(rev_inds(j)) != -1 )
            {
              inds(rev_inds(j)) = -1;
              permutation(N) = j;
              N += 1;
            }
          }

          unsigned tmp = 0;
          for( unsigned kk = N_old; kk < N; ++kk )
          {
            tmp_degrees(tmp) = degree(permutation(kk));
            tmp += 1;
          }
          for( unsigned kk = N_old; kk < N - 1; ++kk )
          {
            tmp = tmp_degrees(kk - N_old);
            const unsigned tmp2 = permutation(kk);
            unsigned ll = kk - 1;

            while( ll >= N_old && tmp_degrees(ll) > tmp )
            {
              tmp_degrees( ll + 1 - N_old ) = tmp_degrees( ll - N_old );
              permutation( ll + 1 ) = permutation(ll);
              ll -= 1;
            }
            tmp_degrees( ll + 1 - N_old ) = tmp;
            permutation( ll + 1 ) = tmp2;
          }
        }

        level_start = level_end;
        level_end = N;
      }
    }
    if( N == permutation.size() )
    {
      break;
    }
  }
}

void SparseMatrixUtilities::cuthillMckee( const SparseMatrixsc& input_matrix, VectorXi& permutation )
{
  assert( input_matrix.rows() == input_matrix.cols() ); assert( input_matrix.cols() == permutation.size() );
  const Eigen::Map<const VectorXi> ind( input_matrix.innerIndexPtr(), input_matrix.nonZeros() );
  const Eigen::Map<const VectorXi> ptr( input_matrix.outerIndexPtr(), input_matrix.nonZeros() + 1 );
  cuthillMckeee( ind, ptr, permutation );
}

void SparseMatrixUtilities::reverseCuthillMckee( const SparseMatrixsc& input_matrix, VectorXi& permutation )
{
  assert( input_matrix.rows() == input_matrix.cols() ); assert( input_matrix.cols() == permutation.size() );
  cuthillMckee( input_matrix, permutation );
  permutation.reverseInPlace();
}

void SparseMatrixUtilities::permuteSparseMatrix( const SparseMatrixsc& input_matrix, const VectorXi& permutation, SparseMatrixsc& output_matrix )
{
  const PermutationMatrix permutation_matrix( permutation );
  ( input_matrix.twistedBy( permutation_matrix.inverse() ) ).evalTo( output_matrix );
}

int SparseMatrixUtilities::computeMatrixBandwidth( const SparseMatrixsc& input_matrix )
{
  int max_bandwidth = 0;
  for( int j = 0; j < input_matrix.outerSize(); ++j )
  {
    for( SparseMatrixsc::InnerIterator it( input_matrix, j ); it; ++it )
    {
      const int bandwidth = std::abs( it.row() - j ) + 1;
      if( bandwidth > max_bandwidth )
      {
        max_bandwidth = bandwidth;
      }
    }
  }
  return max_bandwidth;
}
