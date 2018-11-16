#ifndef SIM_WORKER_H
#define SIM_WORKER_H

#include <QObject>
#include <QApplication>
#include <QDebug>

#include <iostream>

#include "ball2d/Ball2DSim.h"
#include "ball2d/Integrator.h"
#include "ball2d/PythonScripting.h"

class SimWorker : public QObject
{

  Q_OBJECT

public:

  SimWorker()
  : m_sim0()
  , m_sim()
  , m_integrator0()
  , m_integrator()
  , m_scripting()
  , m_iteration( 0 )
  , m_end_time( std::numeric_limits<scalar>::max() )
  , m_H0( 0.0 )
  , m_p0( Vector2s::Zero() )
  , m_L0( 0.0 )
  , m_delta_H0( 0.0 )
  , m_delta_p0( Vector2s::Zero() )
  , m_delta_L0( 0.0 )
  , m_steps_per_frame( 1 )
  {}

  virtual ~SimWorker() override
  {
    std::cout << "Destructor for SimWorker: " << std::endl;
  }

  // TODO: Eliminate this
  unsigned& iteration()
  {
    return m_iteration;
  }
  scalar& endTime()
  {
    return m_end_time;
  }

  const Integrator& integrator() const
  {
    return m_integrator;
  }

  // TODO: Eliminate these
  Integrator& integrator()
  {
    return m_integrator;
  }
  Integrator& integrator0()
  {
    return m_integrator0;
  }

  const Ball2DSim& sim() const
  {
    return m_sim;
  }

  // TODO: Eliminate these
  Ball2DSim& sim()
  {
    return m_sim;
  }
  Ball2DSim& sim0()
  {
    return m_sim0;
  }

  // TODO: Eliminate these
  scalar& H0()
  {
    return m_H0;
  }
  Vector2s& p0()
  {
    return m_p0;
  }
  scalar& L0()
  {
    return m_L0;
  }
  scalar& deltaH0()
  {
    return m_delta_H0;
  }
  Vector2s& deltap0()
  {
    return m_delta_p0;
  }
  scalar& deltaL0()
  {
    return m_delta_L0;
  }

  PythonScripting& scripting()
  {
    return m_scripting;
  }

public slots:

  void takeStep()
  {
    if( m_iteration * scalar( m_integrator.dt() ) >= m_end_time )
    {
      // User-provided end of simulation python callback
      m_scripting.setState( m_sim.state() );
      m_scripting.endOfSimCallback();
      m_scripting.forgetState();
      qInfo( "Simulation complete. Exiting." );
      QApplication::quit();
    }

    const unsigned next_iter{ m_iteration + 1 };

    m_integrator.step( next_iter, m_scripting, m_sim );

    m_iteration++;

    {
      const Ball2DState& state{ m_sim.state() };
      m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - state.computeTotalEnergy() ) );
      const Vector2s p{ state.computeMomentum() };
      m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
      m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
      m_delta_L0 = std::max( m_delta_L0, fabs( m_L0 - state.computeAngularMomentum() ) );
    }

    // const bool was_reset, const bool render, const bool save_image, const int output_num

    constexpr bool was_reset = false;
    const bool fps_multiple = m_iteration % m_steps_per_frame == 0;
    const int output_num = m_iteration / m_steps_per_frame;
    emit postStep( was_reset, fps_multiple, output_num );
  }

  void reset()
  {
    m_sim = m_sim0;
    m_integrator = m_integrator0;

    m_iteration = 0;

    // User-provided start of simulation python callback
    m_scripting.setState( m_sim.state() );
    m_scripting.startOfSimCallback();
    m_scripting.forgetState();

    m_H0 = m_sim.state().computeTotalEnergy();
    m_p0 = m_sim.state().computeMomentum();
    m_L0 = m_sim.state().computeAngularMomentum();
    m_delta_H0 = 0.0;
    m_delta_p0.setZero();
    m_delta_L0 = 0.0;

    constexpr bool was_reset = true;
    const bool fps_multiple = true;
    const int output_num = 0;
    emit postStep( was_reset, fps_multiple, output_num );
  }

  void exportMovieInit()
  {
    constexpr bool was_reset = false;
    const bool fps_multiple = m_iteration == 0;
    const int output_num = m_iteration / m_steps_per_frame;
    emit postStep( was_reset, fps_multiple, output_num );
  }

  // TODO: Rename this
  void setOutputFPS( const int fps )
  {
    if( 1.0 < scalar( m_integrator.dt() * std::intmax_t( fps ) ) )
    {
      qWarning() << "Warning, requested movie frame rate faster than timestep. Dumping at timestep rate.";
      m_steps_per_frame = 1;
    }
    else
    {
      const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_integrator.dt() * std::intmax_t( fps ) ) };
      if( !potential_steps_per_frame.isInteger() )
      {
        if( m_integrator.dt() != Rational<std::intmax_t>{ 0 } )
        {
          qWarning() << "Warning, timestep and output frequency do not yield an integer number of timesteps for data output. Dumping at timestep rate.";
        }
        m_steps_per_frame = 1;
      }
      else
      {
        m_steps_per_frame = unsigned( potential_steps_per_frame.numerator() );
      }
    }

    std::cout << "m_steps_per_frame: " << m_steps_per_frame << std::endl;
  }

signals:

  void postStep( const bool was_reset, const bool fps_multiple, const int output_num );

private:

  // Initial and current state of the simulation
  Ball2DSim m_sim0;
  Ball2DSim m_sim;

  // Initial and current integrators
  Integrator m_integrator0;
  Integrator m_integrator;

  PythonScripting m_scripting;

  // Current iteration of the solver
  unsigned m_iteration;

  // End time of the simulation
  scalar m_end_time;

  // Initial energy, momentum, and angular momentum of the simulation
  scalar m_H0;
  Vector2s m_p0;
  scalar m_L0;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H0;
  Vector2s m_delta_p0;
  scalar m_delta_L0;

  // Number of timesteps between frame outputs
  unsigned m_steps_per_frame;

};

#endif
