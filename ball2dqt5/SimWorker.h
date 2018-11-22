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
  SimWorker( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings, const int dt_display_precision );
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

  const Eigen::VectorXf& bodyColors() const;

  const std::vector<PlaneRenderSettings>& planeRenderSettings() const;

  const std::vector<DrumRenderSettings>& drumRenderSettings() const;

  const std::vector<PortalRenderSettings>& portalRenderSettings() const;

  void computeCameraCenter( bool& sim_empty, Vector4s& bbox ) const;

public slots:

  void reset();

  #ifdef USE_HDF5
  void takeStep( QString movie_dir_name, QString state_dir_name );
  #else
  void takeStep( QString movie_dir_name );
  #endif

  void exportMovieInit();

  #ifdef USE_HDF5
  void exportStateInit( QString dir_name );
  #endif

  void setOutputFPS( const bool lock_output_fps, const bool lock_render_fps, const int fps );

signals:

  void postStep( const bool was_reset, const bool render_frame, const bool save_screenshot, const int output_num );

  void errorMessage( QString message );

private:

  #ifdef USE_HDF5
  void saveStateToHDF5( const QString& dir_name, const int output_num );
  #endif

  Eigen::Vector3f generateColor();
  Eigen::VectorXf generateInitialBodyColors();

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

  // Colors to render bodies in the scene
  std::mt19937_64 m_rn_gen;
  std::vector<Eigen::Vector3f> m_template_colors;
  std::uniform_int_distribution<int> m_color_selector;
  Eigen::VectorXf m_body_colors;

  // Static geometry render instances
  std::vector<PlaneRenderSettings> m_plane_render_settings0;
  std::vector<DrumRenderSettings> m_drum_render_settings0;
  std::vector<PortalRenderSettings> m_portal_render_settings0;

  std::vector<PlaneRenderSettings> m_plane_render_settings;
  std::vector<DrumRenderSettings> m_drum_render_settings;
  std::vector<PortalRenderSettings> m_portal_render_settings;

  // Number of timesteps between outputs of various types
  int m_steps_per_output;
  int m_steps_per_render;

  int m_display_precision;

};

#endif
