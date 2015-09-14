#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QDir>
#include <cstdint>

#include "ball2d/Ball2DSim.h"
#include "scisim/Math/Rational.h"
#include "ball2d/PythonScripting.h"

#include "DisplayController2D.h"
#include "GLCircleRenderer2D.h"

#include <random>

class ImpactMap;
class FrictionSolver;
class ImpactFrictionMap;
class ScriptingCallbackBalls2D;

class GLWidget : public QGLWidget
{

  Q_OBJECT

public:

  GLWidget( QWidget* parent = nullptr );
  ~GLWidget();

  QSize minimumSizeHint() const;
  QSize sizeHint() const;

  bool openScene( const QString& xml_scene_file_name, const bool& render_on_load, unsigned& fps, bool& render_at_fps, bool& lock_camera );

  // Methods to control the solver
  void stepSystem();
  void minimizeSystemsEnergy();
  void resetSystem();

  void getSimData( scalar& time, scalar& T, scalar& U, Vector2s& p, scalar& L );

  void renderAtFPS( const bool render_at_fps );

  void lockCamera( const bool lock_camera );

  void toggleHUD();
  void centerCamera( const bool update_gl = true );

  void saveScreenshot( const QString& file_name );

  void setMovieDir( const QString& dir_name );

  void setMovieFPS( const unsigned fps );

  void exportCameraSettings();

protected:

  void initializeGL();
  void resizeGL( int width, int height );
  void paintGL();

  void mousePressEvent( QMouseEvent* event );
  void mouseReleaseEvent( QMouseEvent* event );
  void mouseMoveEvent( QMouseEvent* event );
  void wheelEvent( QWheelEvent* event );

private:

  bool axesDrawingIsEnabled() const;
  void paintAxes() const;

  void getViewportDimensions( GLint& width, GLint& height ) const;

  void paintSystem() const;

  void paintHUD();

  DisplayController2D m_camera_controller;
  bool m_render_at_fps;
  bool m_lock_camera;
  QPoint m_last_pos;
  bool m_left_mouse_button_pressed;
  bool m_right_mouse_button_pressed;

  GLCircleRenderer2D m_circle_renderer;

  // Colors to render balls in the scene
  VectorXs m_ball_colors;
  std::mt19937_64 m_ball_color_gen;

  // Number of decimal places to display in time display
  int m_display_precision;

  bool m_display_HUD;

  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir; 
  // Number of frames that have been saved in the movie directory
  unsigned m_output_frame;
  // Rate at which to output movie frames
  unsigned m_output_fps;
  // Number of timesteps between frame outputs
  unsigned m_steps_per_frame;

  // Integrator state
  std::unique_ptr<UnconstrainedMap> m_unconstrained_map;
  std::unique_ptr<ImpactOperator> m_impact_operator;
  std::unique_ptr<FrictionSolver> m_friction_solver;
  std::unique_ptr<ImpactFrictionMap> m_if_map;
  std::unique_ptr<ImpactMap> m_imap;
  PythonScripting m_scripting;

  // Current iteration of the solver
  unsigned m_iteration;
  // Current timestep
  Rational<std::intmax_t> m_dt;
  // End time of the simulation
  scalar m_end_time;

  scalar m_CoR;
  scalar m_mu;

  // Initial state of the simulation
  Ball2DState m_state0;

  // Simulation
  Ball2DSim m_sim;

  // Initial energy, momentum, and angular momentum of the simulation
  scalar m_H0;
  Vector2s m_p0;
  scalar m_L0;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H0;
  Vector2s m_delta_p0;
  scalar m_delta_L0;

};

#endif
