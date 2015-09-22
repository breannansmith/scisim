// PythonScripting.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef PYTHON_SCRIPTING_H
#define PYTHON_SCRIPTING_H

#ifdef USE_PYTHON
#include <Python.h>
#endif

#include "scisim/ScriptingCallback.h"
#include "scisim/PythonObject.h"

class Ball2DState;

class PythonScripting final : public ScriptingCallback
{

public:

  PythonScripting();
  PythonScripting( const std::string& path, const std::string& module_name );
  explicit PythonScripting( std::istream& input_stream );

  virtual ~PythonScripting() override = default;

  friend void swap( PythonScripting& first, PythonScripting& second );

  #ifndef USE_PYTHON
  [[noreturn]]
  #endif
  static void initializeCallbacks();

  void setState( Ball2DState& state );
  void forgetState();

  void serialize( std::ostream& output_stream );

private:

  void intializePythonCallbacks();

  virtual void restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor ) override;

  virtual void frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu ) override;

  virtual void startOfSim() override;

  virtual void endOfSim() override;

  virtual void startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) override;

  virtual void endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt ) override;

  virtual std::string name() const override;

  std::string m_path;
  std::string m_module_name;
  #ifdef USE_PYTHON
  PythonObject m_loaded_module;
  PythonObject m_loaded_start_of_step_callback;
  PythonObject m_loaded_end_of_step_callback;
  PythonObject m_loaded_friction_coefficient_callback;
  PythonObject m_loaded_restitution_coefficient_callback;
  #endif

};

#endif
