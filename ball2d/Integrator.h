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

#include "ball2d/PythonScripting.h"

class Ball2DSim;
class PythonScripting;

#ifdef USE_HDF5
class HDF5File;
#endif

class Integrator final
{

public:

  enum class Style
  {
    None,
    Unconstrained,
    Impact,
    ImpactFriction
  };

  Integrator();

  Integrator( const Rational<std::intmax_t>& dt, const std::unique_ptr<UnconstrainedMap>& unconstrained_map,
              const std::unique_ptr<ImpactOperator>& impact_operator, const std::unique_ptr<FrictionSolver>& friction_solver,
              const std::unique_ptr<ImpactMap>& impact_map, const std::unique_ptr<ImpactFrictionMap>& impact_friction_map,
              const scalar& cor, const scalar& mu );

  Integrator( const Integrator& other );
  Integrator( Integrator&& ) = default;

  Integrator& operator=( Integrator other );

  void step( const int next_iter, PythonScripting& scripting, Ball2DSim& sim );

  #ifdef USE_HDF5
  void stepWithForceOutput( const int next_iter, PythonScripting& scripting, Ball2DSim& sim, HDF5File& force_file );
  #endif

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

  const Rational<std::intmax_t>& dt() const;

  const Style& style() const;

private:

  Rational<std::intmax_t> m_dt;
  std::unique_ptr<UnconstrainedMap> m_unconstrained_map;
  std::unique_ptr<ImpactOperator> m_impact_operator;
  std::unique_ptr<FrictionSolver> m_friction_solver;
  std::unique_ptr<ImpactMap> m_impact_map;
  std::unique_ptr<ImpactFrictionMap> m_impact_friction_map;
  scalar m_cor;
  scalar m_mu;
  Style m_style;

};

#endif
