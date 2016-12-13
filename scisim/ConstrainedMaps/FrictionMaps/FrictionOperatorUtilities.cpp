// FrictionOperatorUtilities.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "FrictionOperatorUtilities.h"

#include "scisim/Constraints/Constraint.h"

static void reserveSpaceInBasisMatrix( const int nsamples, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D )
{
  assert( D.cols() % nsamples == 0 );

  const int ncons{ int(D.cols()) / nsamples };

  VectorXi column_nonzeros{ D.cols() };
  auto itr = K.cbegin();
  for( int con_idx = 0; con_idx < ncons; ++con_idx )
  {
    for( int smpl_num = 0; smpl_num < nsamples; ++smpl_num )
    {
      column_nonzeros( con_idx * nsamples + smpl_num ) = (*itr)->frictionStencilSize();
    }
    ++itr;
  }
  assert( itr == K.cend() );
  D.reserve( column_nonzeros );
}

static void buildLinearFrictionBasis( const VectorXs& q, const VectorXs& v, const int nsamples, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.cols() % nsamples == 0 );

  const unsigned ncons{ static_cast<unsigned>( D.cols() ) / nsamples };

  auto itr = K.cbegin();
  for( unsigned con_idx = 0; con_idx < ncons; ++con_idx )
  {
    (*itr)->computeGeneralizedFrictionDisk( q, v, con_idx * nsamples, nsamples, D, drel );
    ++itr;
  }
  assert( itr == K.cend() );
}

void FrictionOperatorUtilities::formGeneralizedFrictionBasis( const VectorXs& q0, const VectorXs& v0, const std::vector<std::unique_ptr<Constraint>>& K, const int num_samples, SparseMatrixsc& D, VectorXs& drel )
{
  assert( num_samples > 0 );
  assert( D.rows() == v0.size() );
  assert( num_samples * int( K.size() ) == D.cols() );

  // Reserve space for entries
  reserveSpaceInBasisMatrix( num_samples, K, D );

  // Build the matrix
  buildLinearFrictionBasis( q0, v0, num_samples, K, D, drel );

  D.prune( []( const Eigen::Index& row, const Eigen::Index& col, const scalar& value ) { return value != 0.0; } );
  assert( D.innerNonZeroPtr() == nullptr );
}

void FrictionOperatorUtilities::formLinearFrictionDiskConstraint( const int num_samples, SparseMatrixsc& E )
{
  {
    const VectorXi column_nonzeros{ VectorXi::Constant( E.cols(), num_samples ) };
    E.reserve( column_nonzeros );
  }
  // For each column
  for( int col = 0; col < E.cols(); ++col )
  {
    for( int samplenum = 0; samplenum < num_samples; ++samplenum )
    {
      // Note the negative for QL
      E.insert( num_samples * col + samplenum, col ) = 1.0;
    }
  }
  E.makeCompressed();
  assert( E.nonZeros() == E.cols() * num_samples );
  assert( fabs( E.sum() - scalar( E.nonZeros() ) ) <= 1.0e-9 );
}
