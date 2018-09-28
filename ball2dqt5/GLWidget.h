#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QDir>

#include <random>

#include "scisim/Math/Rational.h"

#include "ball2d/Ball2DSim.h"
#include "ball2d/PythonScripting.h"

#include "ball2dutils/Ball2DSceneParser.h"

#include "AnnulusShader.h"
#include "AxisShader.h"
#include "CircleShader.h"
#include "PlaneShader.h"
#include "RectangleShader.h"

class QWheelEvent;
class QOpenGLFunctions_3_3_Core;

class UnconstrainedMap;
class ImpactMap;
class ImpactOperator;
class FrictionSolver;
class ImpactFrictionMap;

class GLWidget final : public QOpenGLWidget
{

public:

  explicit GLWidget( QWidget* parent = nullptr );
  virtual ~GLWidget() override;
  GLWidget( GLWidget& ) = delete;
  GLWidget( GLWidget&& ) = delete;
  GLWidget& operator=( const GLWidget& ) = delete;
  GLWidget& operator=( GLWidget&& ) = delete;

  virtual QSize minimumSizeHint() const override;
  virtual QSize sizeHint() const override;


  bool openScene( const QString& xml_scene_file_name, const bool& render_on_load, unsigned& fps, bool& render_at_fps, bool& lock_camera );

  void stepSystem();

  void resetSystem();

  void renderAtFPS( const bool render_at_fps );

  void lockCamera( const bool lock_camera );

  void toggleHUD();

  void centerCamera( const bool update_gl = true );

  void saveScreenshot( const QString& file_name );

  void setMovieDir( const QString& dir_name );

  void setMovieFPS( const unsigned fps );

  void exportCameraSettings();

  void insertBallCallback( const int num_balls );
  void deletePlaneCallback( const int plane_idx );

protected:

  virtual void initializeGL() override;
  virtual void resizeGL( int width, int height ) override;
  virtual void paintGL() override;

  virtual void mousePressEvent( QMouseEvent* event ) override;
  virtual void mouseReleaseEvent( QMouseEvent* event ) override;
  virtual void mouseMoveEvent( QMouseEvent* event ) override;
  virtual void wheelEvent( QWheelEvent* event ) override;

private:

  void copyRenderState();

  bool checkGLErrors() const;

  void paintHUD();

  Vector3s generateColor();

  QOpenGLFunctions_3_3_Core* m_f;

  AxisShader m_axis_shader;
  CircleShader m_circle_shader;
  // TODO: Roll the plane rendering into the rectangle shader
  PlaneShader m_plane_shader;
  AnnulusShader m_annulus_shader;
  RectangleShader m_rectangle_shader;

  // TODO: Pull these into a separate class
  int m_w;
  int m_h;
  float m_display_scale;
  float m_center_x;
  float m_center_y;

  bool m_render_at_fps;
  bool m_lock_camera;
  QPoint m_last_pos;
  bool m_left_mouse_button_pressed;
  bool m_right_mouse_button_pressed;

  // Colors to render balls in the scene
  VectorXs m_ball_colors;
  std::uniform_real_distribution<scalar> m_color_gen;
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
  Ball2DSim m_sim0;
  // Current state of the simulation
  Ball2DSim m_sim;

  // Initial energy, momentum, and angular momentum of the simulation
  scalar m_H0;
  Vector2s m_p0;
  scalar m_L0;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H0;
  Vector2s m_delta_p0;
  scalar m_delta_L0;

  // Static geometry render settings
  std::vector<PlaneRenderSettings> m_plane_render_settings;
  std::vector<DrumRenderSettings> m_drum_render_settings;
  std::vector<PortalRenderSettings> m_portal_render_settings;

  // Global render settings
  int m_num_circle_subdivs;
  int m_num_drum_subdivs;

};

#endif
