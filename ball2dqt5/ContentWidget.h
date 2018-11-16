#ifndef CONTENT_WIDGET_H
#define CONTENT_WIDGET_H

#include <QDir>
#include <QThread>
#include <QWidget>

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

public slots:

  void openUserScene();
  void reloadScene();

  void simulateToggled( const bool state );

  void renderAtFPSToggled( const bool render_at_fps );

  void lockCameraToggled( const bool lock_camera );

  void exportMovieToggled( const bool checked );

  void toggleHUD();
  void toggleCameraLock();
  void centerCamera();
  void toggleControls();
  void toggleFPSLock();

  void toggleSimulating();
  void callReset();
  void callStep();

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

  void initializeUIAndGL( const QString& scene_file_name, const bool render_on_load, const SimSettings& sim_settings,
                          const RenderSettings& render_settings, const SimWorker& sim_worker );

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  void setMovieDir( const QString& dir_name );

  // Qt state

  GLWidget* m_gl_widget;

  QWidget* m_controls_widget;

  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_render_at_fps_checkbox;

  QCheckBox* m_lock_camera_button;

  QCheckBox* m_export_movie_checkbox;

  QSpinBox* m_fps_spin_box;

  QPushButton* m_step_button;

  QPushButton* m_reset_button;

  QPushButton* m_reload_button;

  QPushButton* m_open_button;

  // Threading state

  QThread m_sim_thread;

  SimWorker* m_sim_worker;

  // UI state

  QString m_xml_file_name;

  bool m_render_at_fps;

  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir;

  // Rate at which to output movie frames
  unsigned m_output_fps;

  // Cached state for centering the camera
  bool m_empty;
  Vector4s m_bbox;

};

#endif
