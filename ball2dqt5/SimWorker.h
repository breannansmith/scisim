#ifndef SIM_WORKER_H
#define SIM_WORKER_H

#include <QObject>
#include <QApplication>
#include <QDebug>

// TODO: iostream is temp here
#include <iostream>

#include <random>

#include "ball2dutils/Ball2DSceneParser.h"

#include "ball2d/Ball2DSim.h"
#include "ball2d/Integrator.h"
#include "ball2d/PythonScripting.h"

class SimWorker : public QObject
{

  Q_OBJECT

public:

  SimWorker();

  // TODO: This will become a constructor
  void initialize( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings );

  virtual ~SimWorker() override
  {
    std::cout << "Destructor for SimWorker: " << std::endl;
  }

  void insertBallCallback( const int num_balls );

  void deletePlaneCallback( const int plane_idx );

  unsigned iteration() const
  {
    return m_iteration;
  }

  const scalar& endTime() const
  {
    return m_end_time;
  }

  const Integrator& integrator() const
  {
    return m_integrator;
  }

  const Ball2DSim& sim() const
  {
    return m_sim;
  }

  const scalar& deltaH0() const
  {
    return m_delta_H0;
  }

  const Vector2s& deltap0() const
  {
    return m_delta_p0;
  }

  const scalar& deltaL0() const
  {
    return m_delta_L0;
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

    constexpr bool was_reset = false;
    const bool fps_multiple = m_iteration % m_steps_per_frame == 0;
    const int output_num = m_iteration / m_steps_per_frame;
    emit postStep( was_reset, fps_multiple, output_num );
  }

  void reset();

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

  const VectorXs& ballColors() const
  {
    return m_ball_colors;
  }

  const std::vector<PlaneRenderSettings>& planeRenderSettings() const
  {
    return m_plane_render_settings;
  }

  const std::vector<DrumRenderSettings>& drumRenderSettings() const
  {
    return m_drum_render_settings;
  }

  const std::vector<PortalRenderSettings>& portalRenderSettings() const
  {
    return m_portal_render_settings;
  }

signals:

  void postStep( const bool was_reset, const bool fps_multiple, const int output_num );

private:

  Vector3s generateColor()
  {
    Vector3s color( 1.0, 1.0, 1.0 );
    // Generate colors until we get one with a luminance in [0.1, 0.9]
    while( ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) > 0.9 ||
          ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) < 0.1 )
    {
      color.x() = m_color_gen( m_ball_color_gen );
      color.y() = m_color_gen( m_ball_color_gen );
      color.z() = m_color_gen( m_ball_color_gen );
    }
    return color;
  }

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

  // Colors to render balls in the scene
  VectorXs m_ball_colors;
  std::uniform_real_distribution<scalar> m_color_gen;
  std::mt19937_64 m_ball_color_gen;

  // Static geometry render instances
  std::vector<PlaneRenderSettings> m_plane_render_settings0;
  std::vector<DrumRenderSettings> m_drum_render_settings0;
  std::vector<PortalRenderSettings> m_portal_render_settings0;

  std::vector<PlaneRenderSettings> m_plane_render_settings;
  std::vector<DrumRenderSettings> m_drum_render_settings;
  std::vector<PortalRenderSettings> m_portal_render_settings;

  // Number of timesteps between frame outputs
  unsigned m_steps_per_frame;

};

#endif
