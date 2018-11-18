#ifndef SIM_WORKER_H
#define SIM_WORKER_H

#include <QObject>

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
  SimWorker( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings );
  virtual ~SimWorker() override = default;

  void insertBallCallback( const int num_balls );

  void deletePlaneCallback( const int plane_idx );

  int iteration() const;

  const scalar& endTime() const;

  const Integrator& integrator() const;

  const Ball2DSim& sim() const;

  const scalar& deltaH0() const;

  const Vector2s& deltap0() const;

  const scalar& deltaL0() const;

  const VectorXs& ballColors() const;

  const std::vector<PlaneRenderSettings>& planeRenderSettings() const;

  const std::vector<DrumRenderSettings>& drumRenderSettings() const;

  const std::vector<PortalRenderSettings>& portalRenderSettings() const;

  void computeCameraCenter( bool& sim_empty, Vector4s& bbox ) const;

public slots:

  void reset();

  void takeStep();

  void exportMovieInit();

  void setOutputFPS( const bool use_fps, const int fps );

signals:

  void postStep( const bool was_reset, const bool fps_multiple, const int output_num );

  void errorMessage( const QString& message );

private:

  Vector3s generateColor();

  // Initial and current state of the simulation
  Ball2DSim m_sim0;
  Ball2DSim m_sim;

  // Initial and current integrators
  Integrator m_integrator0;
  Integrator m_integrator;

  PythonScripting m_scripting;

  // Current iteration of the solver
  int m_iteration;

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
  int m_steps_per_frame;

};

#endif
