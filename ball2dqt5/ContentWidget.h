#ifndef CONTENT_WIDGET_H
#define CONTENT_WIDGET_H

#include <QWidget>
#include <QDir>

#include <random>

#include "scisim/Math/MathDefines.h"

#include "ball2d/Ball2DSim.h"
#include "ball2d/Integrator.h"
#include "ball2d/PythonScripting.h"

#include "ball2dutils/Ball2DSceneParser.h"

class QCheckBox;
class QSpinBox;
class GLWidget;

class ContentWidget final : public QWidget
{

  Q_OBJECT

public:

  ContentWidget( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent = nullptr );
  virtual ~ContentWidget() override = default;
  ContentWidget( ContentWidget& ) = delete;
  ContentWidget( ContentWidget&& ) = delete;
  ContentWidget& operator=( const ContentWidget& ) = delete;
  ContentWidget& operator=( ContentWidget&& ) = delete;

  void toggleSimulationCheckbox();
  void disableMovieExport();

  virtual void keyPressEvent( QKeyEvent* event ) override;

  void insertBallCallback( const int num_balls );
  void deletePlaneCallback( const int plane_idx );

public slots:

  void takeStep();
  void resetSystem();

  void openScene();
  void reloadScene();

  void simulateToggled( const bool state );

  void renderAtFPSToggled( const bool render_at_fps );

  void lockCameraToggled( const bool lock_camera );

  void exportMovieToggled( const bool checked );

  void toggleHUD();
  void centerCamera();

  void exportImage();
  void exportMovie();
  void movieFPSChanged( int fps );

  void exportCameraSettings();

private:

  void openScene( const QString& scene_file_name, const bool render_on_load );

  void initializeSimulation( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings );
  void initializeSimAndGL( const QString& scene_file_name, const bool render_on_load, SimSettings& sim_settings, RenderSettings& render_settings );

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  void setMovieFPS( const int fps );

  Vector3s generateColor();

  void setMovieDir( const QString& dir_name );

  // UI state

  GLWidget* m_gl_widget;

  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_render_at_fps_checkbox;

  QCheckBox* m_lock_camera_button;

  QCheckBox* m_export_movie_checkbox;

  QSpinBox* m_fps_spin_box;

  // Sim and rendering state

  QString m_xml_file_name;

  bool m_simulate_toggled;

  bool m_render_at_fps;

  // Colors to render balls in the scene
  VectorXs m_ball_colors;
  std::uniform_real_distribution<scalar> m_color_gen;
  std::mt19937_64 m_ball_color_gen;

  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir;
  // Number of frames that have been saved in the movie directory
  unsigned m_output_frame;
  // Rate at which to output movie frames
  unsigned m_output_fps;
  // Number of timesteps between frame outputs
  unsigned m_steps_per_frame;

  // Current iteration of the solver
  unsigned m_iteration;
  // End time of the simulation
  scalar m_end_time;

  // Initial and current state of the simulation
  Ball2DSim m_sim0;
  Ball2DSim m_sim;

  // Initial and current integrators
  Integrator m_integrator0;
  Integrator m_integrator;

  PythonScripting m_scripting;

  // Initial energy, momentum, and angular momentum of the simulation
  scalar m_H0;
  Vector2s m_p0;
  scalar m_L0;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H0;
  Vector2s m_delta_p0;
  scalar m_delta_L0;

  // Static geometry render instances
  std::vector<PlaneRenderSettings> m_plane_render_settings0;
  std::vector<DrumRenderSettings> m_drum_render_settings0;
  std::vector<PortalRenderSettings> m_portal_render_settings0;

  std::vector<PlaneRenderSettings> m_plane_render_settings;
  std::vector<DrumRenderSettings> m_drum_render_settings;
  std::vector<PortalRenderSettings> m_portal_render_settings;

};

#endif
