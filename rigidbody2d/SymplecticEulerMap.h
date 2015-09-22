// SymplecticEulerMap.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef SYMPLECTIC_EULER_MAP_H
#define SYMPLECTIC_EULER_MAP_H

#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

class SymplecticEulerMap final : public UnconstrainedMap
{

public:

  SymplecticEulerMap() = default;
  explicit SymplecticEulerMap( std::istream& input_stream );

  virtual ~SymplecticEulerMap() override = default;

  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

};

#endif
