// ExponentialEulerMap.h
//
// Breannan Smith
// Last updated: 09/15/2015

// TODO: This is actually projected euler, rename the class

#ifndef EXPONENTIAL_EULER_MAP_H
#define EXPONENTIAL_EULER_MAP_H

#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

class ExponentialEulerMap final : public UnconstrainedMap
{

public:

  virtual ~ExponentialEulerMap() override;

  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

};

#endif
