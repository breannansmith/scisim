#include "FrictionOperator.h"

#include "scisim/Constraints/Constraint.h"

FrictionOperator::~FrictionOperator()
{}

// TODO: Despecialize from smooth
void FrictionOperator::formGeneralizedSmoothFrictionBasis( const unsigned ndofs, const unsigned ncons, const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& K, const MatrixXXsc& bases, SparseMatrixsc& D )
{
  assert( ncons == K.size() );

  const unsigned nambientdims{ static_cast<unsigned>( bases.rows() ) };
  const unsigned nsamples{ nambientdims - 1 };

  D.resize( ndofs, nsamples * ncons );

  auto itr = K.cbegin();
  {
    VectorXi column_nonzeros( D.cols() );
    for( unsigned collision_number = 0; collision_number < ncons; ++collision_number )
    {
      for( unsigned sample_number = 0; sample_number < nsamples; ++sample_number )
      {
        assert( nsamples * collision_number + sample_number < column_nonzeros.size() );
        column_nonzeros( nsamples * collision_number + sample_number ) = (*itr)->frictionStencilSize();
      }
      ++itr;
    }
    assert( ( column_nonzeros.array() > 0 ).all() );
    assert( itr == K.cend() );
    D.reserve( column_nonzeros );
  }

  itr = K.cbegin();
  for( unsigned collision_number = 0; collision_number < ncons; ++collision_number )
  {
    for( unsigned sample_number = 0; sample_number < nsamples; ++sample_number )
    {
      const unsigned current_column{ nsamples * collision_number + sample_number };
      const VectorXs current_sample{ bases.col( nambientdims * collision_number + sample_number + 1 ) };
      assert( fabs( current_sample.dot( bases.col( nambientdims * collision_number ) ) ) <= 1.0e-6 );
      (*itr)->computeGeneralizedFrictionGivenTangentSample( q, current_sample, current_column, D );
    }
    ++itr;
  }
  assert( itr == K.cend() );

  D.prune( []( const Eigen::Index& /*row*/, const Eigen::Index& /*col*/, const scalar& value ) { return value != 0.0; } );
  assert( D.innerNonZeroPtr() == nullptr );
}

#include <iostream>

void FrictionOperator::formGeneralizedFrictionBasis( const VectorXs& /*q*/, const VectorXs& /*v*/, const std::vector<std::unique_ptr<Constraint>>& /*K*/, SparseMatrixsc& /*D*/, VectorXs& /*drel*/ )
{
  std::cerr << "Deprecated method FrictionOperator::formGeneralizedFrictionBasis not implemented for " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}
