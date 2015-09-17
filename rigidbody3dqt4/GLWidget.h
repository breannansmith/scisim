#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QDir>
#include <cstdint>

#include "scisim/Math/MathDefines.h"
#include "scisim/Math/Rational.h"

#include "rigidbody3d/RigidBody3DSim.h"
#include "rigidbody3d/PythonScripting.h"

#include "PerspectiveCameraController.h"
#include "OrthographicCameraController.h"

#include "Rendering/StaticCylinderRenderer.h"
#include "Rendering/StaticPlaneRenderer.h"
#include "Rendering/PlanarPortalRenderer.h"

class OpenGL3DSphereRenderer;
class BodyGeometryRenderer;
class RigidBodyScriptingCallback;
class RenderingState;

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

  // Doubles because the plotting library expects doubles...
  void getSimData( double& time, double& T, double& U, Eigen::Vector3d& p, Eigen::Vector3d& L );

  void renderAtFPS( const bool render_at_fps );
  void lockCamera( const bool lock_camera );
  void toggleHUD();
  void toggleXYGrid();
  void toggleYZGrid();
  void toggleXZGrid();
  void centerCamera();

  void saveScreenshot( const QString& file_name );

  void setMovieDir( const QString& dir_name );

  void setMovieFPS( const unsigned fps );
  
  void exportCameraSettings();

  void enablePerspectiveCamera();
  void enableOrthographicXYCamera();
  void enableOrthographicZYCamera();
  void enableOrthographicZXCamera();

protected:

  void initializeGL();
  void resizeGL( int width, int height );
  void paintGL();

  void mousePressEvent( QMouseEvent* event );
  void mouseReleaseEvent( QMouseEvent* event );
  void mouseMoveEvent( QMouseEvent* event );
  void wheelEvent( QWheelEvent* event );

private:

  void initializeRenderingSettings( const RenderingState& rendering_state );

  bool axesDrawingIsEnabled() const;
  void paintAxes() const;
  void paintXYRulers() const;
  void paintYZRulers() const;
  void paintXZRulers() const;

  void getViewportDimensions( GLint& width, GLint& height ) const;

  void paintSphere( const RigidBodySphere& sphere, const Vector3s& color ) const;
  void paintBox( const RigidBodyBox& box, const Vector3s& color ) const;
  void paintTriangleMesh( const RigidBodyTriangleMesh& mesh, const Vector3s& color ) const;
  void paintBody( const int geo_idx, const RigidBodyGeometry& geometry, const Vector3s& color ) const;
  void paintSystem() const;

  void paintHUD();

  bool m_use_perspective_camera;
  PerspectiveCameraController m_perspective_camera_controller;
  OrthographicCameraController m_orthographic_controller;

  bool m_render_at_fps;
  bool m_lock_camera;
  QPoint m_last_pos;
  bool m_left_mouse_button_pressed;
  bool m_right_mouse_button_pressed;

  // Colors to render balls in the scene
  // TODO: Change this into an std::vector
  VectorXs m_body_colors;

  // Number of decimal places to display in time display
  int m_display_precision;

  bool m_display_HUD;
  bool m_display_xy_grid;
  bool m_display_yz_grid;
  bool m_display_xz_grid;


  // Directory to save periodic screenshots of the simulation into
  QString m_movie_dir_name;
  QDir m_movie_dir; 
  // Number of frames that have been saved in the movie directory
  unsigned m_output_frame;
  // Rate at which to output movie frames
  unsigned m_output_fps;
  // Number of timesteps between frame outputs
  unsigned m_steps_per_frame;

  // Simulation state
  std::unique_ptr<UnconstrainedMap> m_unconstrained_map;
  std::unique_ptr<ImpactOperator> m_impact_operator;
  std::unique_ptr<FrictionSolver> m_friction_solver;
  std::unique_ptr<ImpactFrictionMap> m_impact_friction_map;
  PythonScripting m_scripting;

  // Current iteration of the solver
  unsigned m_iteration;
  // Current timestep
  Rational<std::intmax_t> m_dt;
  // End time of the simulation
  scalar m_end_time;

  scalar m_CoR;
  scalar m_mu;

  // Simulation
  RigidBody3DSim m_sim;
  RigidBody3DState m_sim_state_backup;

  // Initial energy, momentum, and angular momentum of the simulation
  scalar m_H0;
  Vector3s m_p0;
  Vector3s m_L0;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H0;
  Vector3s m_delta_p0;
  Vector3s m_delta_L0;

  std::unique_ptr<OpenGL3DSphereRenderer> m_sphere_renderer;
  std::vector<StaticPlaneRenderer> m_plane_renderers;
  std::vector<StaticCylinderRenderer> m_cylinder_renderers;
  std::vector<PlanarPortalRenderer> m_portal_renderers;
  std::vector<std::unique_ptr<BodyGeometryRenderer>> m_body_renderers;

};

#endif
