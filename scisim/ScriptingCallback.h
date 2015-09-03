// ScriptingCallback.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef SCRIPTING_CALLBACK_H
#define SCRIPTING_CALLBACK_H

#include "SCISim/Math/MathDefines.h"

#include <memory>

class Constraint;
template<typename T> class Rational;

class ScriptingCallback
{

public:

  virtual ~ScriptingCallback() = 0;

  void restitutionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor );

  void frictionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu );

  void startOfSimCallback();

  void endOfSimCallback();

  void startOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt );

  void endOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt );

private:

  virtual void restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor ) = 0;

  virtual void frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu ) = 0;

  virtual void startOfSim() = 0;

  virtual void endOfSim() = 0;

  virtual void startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) = 0;

  virtual void endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) = 0;

  virtual std::string name() const = 0;

};

#endif
