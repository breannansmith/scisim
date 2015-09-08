// StaticDrum.h
//
// Breannan Smith
// Last updated: 09/07/2015

#ifndef STATIC_DRUM_H
#define STATIC_DRUM_H

#include "SCISim/Math/MathDefines.h"

class StaticDrum final
{

public:

  StaticDrum( const Vector2s& x, const scalar& r );
  StaticDrum( std::istream& input_stream );

  const Vector2s& x() const;

  const scalar& r() const;

  void serialize( std::ostream& output_stream ) const;

private:

  const Vector2s m_x;
  const scalar m_r;

};

#endif
