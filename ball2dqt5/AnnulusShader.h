#ifndef ANNULUS_SHADER_H
#define ANNULUS_SHADER_H

#include <QOpenGLFunctions_3_3_Core>
#include <Eigen/Core>

class QMatrix4x4;

class AnnulusShader final
{

public:

  AnnulusShader();

  AnnulusShader( const AnnulusShader& ) = delete;
  AnnulusShader( AnnulusShader&& ) = delete;
  AnnulusShader& operator=( const AnnulusShader& ) = delete;
  AnnulusShader& operator=( AnnulusShader&& ) = delete;

  void initialize( QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& annulusData();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_instance_VBO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // Storage for the annulus to render
  // Size is: 4 * num_annuli
  // center_of_mass radius0 radius1
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_annulus_data;

  bool m_data_buffered;
  int m_last_copied_size;

};

#endif
