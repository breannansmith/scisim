#ifndef DISCRETE_INTEGRATOR_H
#define DISCRETE_INTEGRATOR_H

#include <memory>

#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/Math/MathDefines.h"
#include "scisim/Math/Rational.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "rigidbody3d/PythonScripting.h"

class Ball2DSim;

class Integrator final
{

public:

  Integrator( const Rational<std::intmax_t>& dt, const std::unique_ptr<UnconstrainedMap>& unconstrained_map, const std::unique_ptr<ImpactOperator>& impact_operator,
              const std::unique_ptr<FrictionSolver>& friction_solver, const std::unique_ptr<ImpactMap>& impact_map, const std::unique_ptr<ImpactFrictionMap>& impact_friction_map,
              const scalar& cor, const scalar& mu );

//   DiscreteIntegrator(const DiscreteIntegrator& other);
//   DiscreteIntegrator( DiscreteIntegrator&& ) = default;

//   DiscreteIntegrator& operator=( DiscreteIntegrator other );
//   DiscreteIntegrator& operator=( DiscreteIntegrator&& ) = default;

//   ~DiscreteIntegrator() = default;

//   void setPythonCallback( const std::string& path, const std::string& module_name );
//   void pythonStartOfSim( RigidBody3DSim& sim );
//   void pythonEndOfSim( RigidBody3DSim& sim );

//   const Rational<std::intmax_t>& timestep() const;

  void step( const int next_iter, Ball2DSim& sim );

//   scalar computeTime() const;
  
//   std::unique_ptr<ImpactFrictionMap>& impactFrictionMap();
//   ImpactFrictionMap* impactFrictionMapPointer();
  
//   bool frictionIsEnabled() const;

//   //ImpactFrictionMap& impactFrictionMap();

//   void serialize( std::ostream& output_stream ) const;
//   void deserialize( std::istream& input_stream );

private:

  Rational<std::intmax_t> m_dt;
  std::unique_ptr<UnconstrainedMap> m_unconstrained_map;
  std::unique_ptr<ImpactOperator> m_impact_operator;
  std::unique_ptr<FrictionSolver> m_friction_solver;
  std::unique_ptr<ImpactMap> m_impact_map;
  std::unique_ptr<ImpactFrictionMap> m_impact_friction_map;
  scalar m_cor;
  scalar m_mu;
  PythonScripting m_scripting;

};

#endif
