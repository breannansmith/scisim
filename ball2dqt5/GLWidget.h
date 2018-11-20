#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>

#include "scisim/Math/MathDefines.h"

#include "ball2dutils/Ball2DSceneParser.h"

#include "AnnulusShader.h"
#include "AxisShader.h"
#include "CircleShader.h"
#include "PlaneShader.h"
#include "RectangleShader.h"

class QOpenGLFunctions_3_3_Core;
class QWheelEvent;

class Ball2DState;

class GLWidget final : public QOpenGLWidget
{

public:

  GLWidget( QWidget* parent, const QSurfaceFormat& format );
  virtual ~GLWidget() override;
  GLWidget( GLWidget& ) = delete;
  GLWidget( GLWidget&& ) = delete;
  GLWidget& operator=( const GLWidget& ) = delete;
  GLWidget& operator=( GLWidget&& ) = delete;

  virtual QSize minimumSizeHint() const override;
  virtual QSize sizeHint() const override;

  int sampleCount() const;

  void initialize( const bool& render_on_load, const RenderSettings& render_settings, const int dt_display_precision,
                   const Ball2DState& state, const Eigen::VectorXf& body_colors, const std::vector<PlaneRenderSettings>& plane_settings,
                   const std::vector<DrumRenderSettings>& drum_settings, const std::vector<PortalRenderSettings>& portal_settings,
                   const scalar& end_time, const bool empty, const Vector4s& bbox );

  void setBackgroundColor( const Eigen::Matrix<GLfloat, 3, 1>& background_color );

  void setHUDTextColor( const Eigen::Matrix<GLfloat, 3, 1>& text_color );

  void lockCamera( const bool lock_camera );

  void toggleHUD();

  void centerCamera( const bool update_gl, const bool empty, const Vector4s& bbox );

  void saveScreenshot( const QString& file_name );

  std::string exportCameraSettings( const int output_fps, const bool render_at_fps );

  void copyRenderState( const Ball2DState& state, const Eigen::VectorXf& body_colors, const std::vector<PlaneRenderSettings>& plane_settings,
                        const std::vector<DrumRenderSettings>& drum_settings, const std::vector<PortalRenderSettings>& portal_settings,
                        const scalar& time, const scalar& end_time, const scalar& delta_H, const Vector2s& delta_p, const scalar& delta_L );

protected:

  virtual void initializeGL() override;
  virtual void resizeGL( int width, int height ) override;
  virtual void paintGL() override;

  virtual void mousePressEvent( QMouseEvent* event ) override;
  virtual void mouseReleaseEvent( QMouseEvent* event ) override;
  virtual void mouseMoveEvent( QMouseEvent* event ) override;
  virtual void wheelEvent( QWheelEvent* event ) override;

private:

  bool checkGLErrors() const;

  void paintHUD();

  QOpenGLFunctions_3_3_Core* m_f;

  AxisShader m_axis_shader;
  CircleShader m_circle_shader;
  // TODO: Roll the plane rendering into the rectangle shader
  PlaneShader m_plane_shader;
  AnnulusShader m_annulus_shader;
  RectangleShader m_rectangle_shader;

  int m_w;
  int m_h;
  float m_display_scale;
  float m_center_x;
  float m_center_y;

  bool m_lock_camera;
  QPoint m_last_pos;
  bool m_left_mouse_button_pressed;
  bool m_right_mouse_button_pressed;

  // Number of decimal places to display in time display
  int m_display_precision;

  bool m_display_HUD;

  scalar m_time;
  scalar m_end_time;

  // Max change in energy, momentum, and angular momentum
  scalar m_delta_H;
  Vector2s m_delta_p;
  scalar m_delta_L;

  // Global render settings
  int m_num_circle_subdivs;
  int m_num_drum_subdivs;
  int m_num_aa_samples;

  Eigen::Matrix<GLfloat, 3, 1> m_bg_color;
  Eigen::Matrix<GLfloat, 3, 1> m_hud_text_color;

};

#endif
