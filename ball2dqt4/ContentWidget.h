#ifndef CONTENT_WIDGET_H
#define CONTENT_WIDGET_H

#include <QWidget>

class QCheckBox;
class QSpinBox;
class GLWidget;

class ContentWidget final : public QWidget
{

  Q_OBJECT

public:

  ContentWidget( const QString& scene_name, QWidget* parent = nullptr );
  virtual ~ContentWidget() override = default;

  void toggleSimulationCheckbox();
  void disableMovieExport();

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

  QString getOpenFileNameFromUser( const QString& prompt );
  QString getSaveFileNameFromUser( const QString& prompt );
  QString getDirectoryNameFromUser( const QString& prompt );

  QTimer* m_idle_timer;
  GLWidget* m_gl_widget;
  QCheckBox* m_simulate_checkbox;

  QCheckBox* m_render_at_fps_checkbox;

  QCheckBox* m_lock_camera_button;

  QCheckBox* m_export_movie_checkbox;

  QString m_xml_file_name;

  QSpinBox* m_fps_spin_box;

};

#endif
