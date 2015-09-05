// StaticDrum.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef STATIC_DRUM_H
#define STATIC_DRUM_H

// TODO: Make member vars const

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

  Vector2s m_x;
  scalar m_r;

};

#endif
