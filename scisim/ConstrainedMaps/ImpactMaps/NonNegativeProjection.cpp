#include "NonNegativeProjection.h"

void NonNegativeProjection::operator()( VectorXs& x ) const
{
  for( int con_num = 0; con_num < x.size(); ++con_num )
  {
    if( x( con_num ) < 0.0 )
    {
      x( con_num ) = 0.0;
    }
  }
}
