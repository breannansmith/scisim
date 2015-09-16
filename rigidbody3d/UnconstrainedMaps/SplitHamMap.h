// SplitHamMap.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef SPLIT_HAM_MAP_H
#define SPLIT_HAM_MAP_H

#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

class SplitHamMap final : public UnconstrainedMap
{

public:

  virtual ~SplitHamMap() override;

  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) override;

  virtual void linearInertialConfigurationUpdate( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 ) override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

};

#endif
