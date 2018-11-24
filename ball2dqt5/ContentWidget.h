#ifndef CONTENT_WIDGET_H
#define CONTENT_WIDGET_H

#include <QDir>
#include <QThread>
#include <QWidget>

#include "ball2dutils/Ball2DSceneParser.h"

#include "scisim/Math/MathDefines.h"

class QCheckBox;
// class QPushButton;

class GLWidget;
class SimWorker;
class ValidatingSpinBox;

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

  QAction* resetAction();
  QAction* stepAction();
  QAction* simulateAction();

  // TODO: This is kind of hacky, but it works for now. Needed because failing to load a directory can force a retoggle.
  void wireSaveMovieAction( QAction* movie_action );

  #ifdef USE_HDF5
  void wireSaveStateAction( QAction* state_action );
  #endif

  void wireToggleHUD( QAction* hud ) const;

  bool isCameraLocked() const;
  void wireCameraLocked( QAction* locked ) const;

  bool isLockRenderFPSChecked() const;
  bool isLockRenderFPSEnabled() const;
  void wireLockRenderFPS( QAction* locked ) const;

  bool isLockOutputFPSChecked() const;
  bool isLockOutputFPSEnabled() const;
  void wireLockOutputFPS( QAction* locked ) const;

public slots:

  void openUserScene();
  void reloadScene();

  void outputFPSToggled();

  void lockRenderFPSToggled( const bool lock_render_fps );

  void exportMovieToggled( const bool checked );

  #ifdef USE_HDF5
  void exportStateToggled( const bool checked );
  #endif

  void toggleHUDCheckbox();
  void toggleCameraLockCheckbox();
  void toggleRenderFPSLockCheckbox();
  void toggleOutputFPSLockCheckbox();

  void centerCamera();
  void toggleControls();

  void exportImage();
  void exportMovie();
  #ifdef USE_HDF5
  void exportState();
  #endif

  void exportCameraSettings();

  void copyStepResults( const bool was_reset, const bool render_frame, const bool save_screenshot, const int output_num );

  void workerErrorMessage( QString message );

signals:

  #ifdef USE_HDF5
  void stepSimulation( QString movie_dir_name, QString state_dir_name );
  #else
  void stepSimulation( QString movie_dir_name );
  #endif

  void outputFPSChanged( const bool lock_output_fps, const bool lock_render_fps, const int fps );
  void exportMovieEnabled();
  #ifdef USE_HDF5
  void exportStateEnabled( QString dir_name );
  #endif

  void lockRenderFPSCheckboxEnabled( const bool enabled );
  void lockOutputFPSCheckboxEnabled( const bool enabled );

private:

  void setFPS( const int fps );

  void wireSimWorker();

  void disableMovieExport();

  #ifdef USE_HDF5
  void disableStateExport();
  #endif

  void openScene( const QString& scene_file_name );

  void initializeUIAndGL( const QString& scene_file_name, const bool render_on_load,
                          const SimSettings& sim_settings, const RenderSettings& render_settings );

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  void setMovieDir( const QString& dir_name );

  #ifdef USE_HDF5
  void setStateDir( const QString& dir_name );
  #endif

  // Qt state

  GLWidget* m_gl_widget;

  QWidget* m_controls_widget;

  QAction* m_simulate_action;
  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_lock_render_fps_checkbox;

  QCheckBox* m_lock_camera_checkbox;

  QCheckBox* m_export_movie_checkbox;

  #ifdef USE_HDF5
  QCheckBox* m_export_state_checkbox;
  #endif

  QCheckBox* m_lock_output_fps_checkbox;

  QCheckBox* m_display_hud_checkbox;

  ValidatingSpinBox* m_fps_spin_box;

  QAction* m_step_action;

  QAction* m_reset_action;

  // Threading state

  QThread m_sim_thread;

  SimWorker* m_sim_worker;

  // UI state

  QString m_xml_file_name;

  bool m_lock_render_fps;

  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir;

  #ifdef USE_HDF5
  // Directory to save periodic state snapshots of the simulation into
  QString m_state_dir_name;
  #endif

  // Rate at which to output movie frames
  bool m_lock_output_fps;
  int m_output_fps;

  // Cached state for centering the camera
  bool m_empty;
  Vector4s m_bbox;

};

#endif
