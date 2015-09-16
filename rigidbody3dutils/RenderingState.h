// RenderingState.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef RENDERING_STATE
#define RENDERING_STATE

#include "SCISim/Math/MathDefines.h"

#include <vector>

class PlaneRendererState final
{

public:

  PlaneRendererState( const unsigned plane_index, const Array2s& r );

  unsigned index() const;
  void setIndex( const unsigned plane_index );
  const Array2s& r() const;
  bool operator<( const PlaneRendererState& rhs ) const;

private:

  unsigned m_plane_index;
  Array2s m_r;

};


class CylinderRendererState final
{

public:

  CylinderRendererState( const unsigned cylinder_index, const scalar& L );

  unsigned index() const;
  const scalar& L() const;

private:

  unsigned m_cylinder_index;
  scalar m_L;

};


class PlanarPortalRendererState final
{

public:

  PlanarPortalRendererState( const unsigned portal_idx, const Array2s& half_width0, const Array2s& half_width1 );

  const unsigned& portalIndex() const;
  const Array2s& halfWidth0() const;
  const Array2s& halfWidth1() const;

private:

  unsigned m_portal_index;
  Array2s m_half_width0;
  Array2s m_half_width1;

};

class RenderingState final
{

public:

  RenderingState();

  void addPlaneRenderer( const unsigned plane_index, const Array2s& r );
  void addCylinderRenderer( const unsigned cylinder_index, const scalar& L );
  void addPortalRenderer( const unsigned portal_index, const Array2s& half_width0, const Array2s& half_width1 );

  void setPerspectvieCameraSettings( const scalar& theta, const scalar& phi, const scalar& rho, const Vector3s& lookat, const Vector3s& up );
  void setOrthographicCameraSettings( const std::string& projection_plane, const Vector3s& x, const scalar& scale );

  void setFPS( const unsigned fps );
  void setRenderAtFPS( const bool render_at_fps );
  void setLocked( const bool locked );

  std::vector<PlaneRendererState>::size_type numPlaneRenderers() const;
  const PlaneRendererState& planeRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index ) const;
  PlaneRendererState& planeRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index );
  std::vector<PlaneRendererState>& planeRenderers();
  // Sort the plane renderers by plane index
  void sortPlaneRenderers();
  void deleteStaticPlaneRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index );

  std::vector<CylinderRendererState>::size_type numCylinderRenderers() const;
  const CylinderRendererState& cylinderRenderer( const std::vector<CylinderRendererState>::size_type cylinder_renderer_index ) const;

  std::vector<PlanarPortalRendererState>::size_type numPlanarPortalRenderers() const;
  const PlanarPortalRendererState& portalRenderer( const std::vector<PlanarPortalRendererState>::size_type portal_renderer_index ) const;

  bool cameraSettingsInitialized() const;
  bool perspectiveCameraSelected() const;
  bool orthographicCameraSelected() const;

  const Vector3s& cameraUp() const;
  const scalar& cameraTheta() const;
  const scalar& cameraPhi() const;
  const scalar& cameraRho() const;
  const Vector3s& cameraLookAt() const;

  const std::string& orthographicProjectionPlane() const;
  const Vector3s& orthographicX() const;
  const scalar& orthographicScale() const;

  unsigned FPS() const;
  bool renderAtFPS() const;
  bool locked() const;

private:

  std::vector<PlaneRendererState> m_plane_renderers;
  std::vector<CylinderRendererState> m_cylinder_renderers;
  std::vector<PlanarPortalRendererState> m_portal_renderers;

  bool m_use_perspective_camera;
  scalar m_camera_theta;
  scalar m_camera_phi;
  scalar m_camera_rho;
  Vector3s m_camera_lookat;
  Vector3s m_camera_up;

  bool m_use_orthographic_camera;
  std::string m_ortho_projection_plane;
  Vector3s m_ortho_x;
  scalar m_ortho_scale;

  unsigned m_fps;
  bool m_render_at_fps;
  bool m_locked;

};

#endif
