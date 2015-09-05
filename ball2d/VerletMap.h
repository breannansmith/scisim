// VerletMap.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef VERLET_MAP_H
#define VERLET_MAP_H

#include "SCISim/UnconstrainedMaps/UnconstrainedMap.h"

class VerletMap final : public UnconstrainedMap
{

public:

  VerletMap();
  VerletMap( std::istream& input_stream );
  virtual ~VerletMap() override;

  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

};

#endif
