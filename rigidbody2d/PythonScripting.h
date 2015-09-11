// PythonScripting.h
//
// Breannan Smith
// Last updated: 09/10/2015

#ifndef PYTHON_SCRIPTING_H
#define PYTHON_SCRIPTING_H

#include "SCISim/ScriptingCallback.h"

class PythonScripting final : public ScriptingCallback
{

public:

  PythonScripting();

  virtual ~PythonScripting() override = default;

private:

  virtual void restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor ) override;

  virtual void frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu ) override;

  virtual void startOfSim() override;

  virtual void endOfSim() override;
  
  virtual void startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) override;

  virtual void endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) override;

  virtual std::string name() const override;

};

#endif
