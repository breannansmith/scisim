#ifndef CONTENT_WIDGET_H
#define CONTENT_WIDGET_H

#include <QDir>
#include <QThread>
#include <QWidget>

#include <random>

#include "scisim/Math/MathDefines.h"

#include "ball2dutils/Ball2DSceneParser.h"

class QCheckBox;
class QPushButton;
class QSpinBox;

class GLWidget;
class SimWorker;

class ContentWidget final : public QWidget
{

  Q_OBJECT

public:

  ContentWidget( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent = nullptr );
  virtual ~ContentWidget() override;
  ContentWidget( ContentWidget& ) = delete;
  ContentWidget( ContentWidget&& ) = delete;
  ContentWidget& operator=( const ContentWidget& ) = delete;
  ContentWidget& operator=( ContentWidget&& ) = delete;

  virtual void keyPressEvent( QKeyEvent* event ) override;

  void insertBallCallback( const int num_balls );
  void deletePlaneCallback( const int plane_idx );

public slots:

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
  void setMovieFPS( const int fps );

  void exportCameraSettings();

  void copyStepResults( const bool was_reset, const bool fps_multiple, const int output_num );

signals:

  void resetSimulation();
  void stepSimulation();

  void outputFPSChanged( const int fps );
  void exportEnabled();

private:

  void wireSimWorker();

  void disableMovieExport();

  void openScene( const QString& scene_file_name, const bool render_on_load );

  void initializeSimulation( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings, SimWorker& sim_worker );
  void initializeSimAndGL( const QString& scene_file_name, const bool render_on_load, SimSettings& sim_settings, RenderSettings& render_settings, SimWorker& sim_worker );

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  Vector3s generateColor();

  void setMovieDir( const QString& dir_name );

  // UI state

  GLWidget* m_gl_widget;

  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_render_at_fps_checkbox;

  QCheckBox* m_lock_camera_button;

  QCheckBox* m_export_movie_checkbox;

  QSpinBox* m_fps_spin_box;

  QPushButton* m_step_button;

  QPushButton* m_reset_button;

  // Threading state

  QThread m_sim_thread;

  SimWorker* m_sim_worker;

  // Sim and rendering state

  QString m_xml_file_name;

  bool m_render_at_fps;

  // Colors to render balls in the scene
  VectorXs m_ball_colors;
  std::uniform_real_distribution<scalar> m_color_gen;
  std::mt19937_64 m_ball_color_gen;

  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir;
  // Rate at which to output movie frames
  unsigned m_output_fps;

  // Static geometry render instances
  std::vector<PlaneRenderSettings> m_plane_render_settings0;
  std::vector<DrumRenderSettings> m_drum_render_settings0;
  std::vector<PortalRenderSettings> m_portal_render_settings0;

  std::vector<PlaneRenderSettings> m_plane_render_settings;
  std::vector<DrumRenderSettings> m_drum_render_settings;
  std::vector<PortalRenderSettings> m_portal_render_settings;

  // Cached state for centering the camera
  bool m_empty;
  Vector4s m_bbox;

};

#endif
