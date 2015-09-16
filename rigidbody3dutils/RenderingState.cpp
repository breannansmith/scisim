// RenderingState.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "RenderingState.h"

PlaneRendererState::PlaneRendererState( const unsigned plane_index, const Array2s& r )
: m_plane_index( plane_index )
, m_r( r )
{
  assert( ( m_r > 0.0 ).all() );
}

unsigned PlaneRendererState::index() const
{
  return m_plane_index;
}

void PlaneRendererState::setIndex( const unsigned plane_index )
{
  m_plane_index = plane_index;
}

const Array2s& PlaneRendererState::r() const
{
  return m_r;
}

bool PlaneRendererState::operator<( const PlaneRendererState& rhs ) const
{
  return m_plane_index < rhs.m_plane_index;
}

CylinderRendererState::CylinderRendererState( const unsigned cylinder_index, const scalar& L )
: m_cylinder_index( cylinder_index )
, m_L( L )
{
  assert( m_L > 0.0 );
}

unsigned CylinderRendererState::index() const
{
  return m_cylinder_index;
}

const scalar& CylinderRendererState::L() const
{
  return m_L;
}


PlanarPortalRendererState::PlanarPortalRendererState( const unsigned portal_idx, const Array2s& half_width0, const Array2s& half_width1 )
: m_portal_index( portal_idx )
, m_half_width0( half_width0 )
, m_half_width1( half_width1 )
{
  assert( ( m_half_width0 >= 0.0 ).all() );
  assert( ( m_half_width1 >= 0.0 ).all() );
}

const unsigned& PlanarPortalRendererState::portalIndex() const
{
  return m_portal_index;
}

const Array2s& PlanarPortalRendererState::halfWidth0() const
{
  return m_half_width0;
}

const Array2s& PlanarPortalRendererState::halfWidth1() const
{
  return m_half_width1;
}

RenderingState::RenderingState()
: m_plane_renderers()
, m_cylinder_renderers()
, m_portal_renderers()
, m_use_perspective_camera( false )
, m_camera_theta()
, m_camera_phi()
, m_camera_rho()
, m_camera_lookat()
, m_camera_up()
, m_use_orthographic_camera( false )
, m_ortho_projection_plane( "xy" )
, m_ortho_x( Vector3s::Zero() )
, m_ortho_scale( 1.0 )
, m_fps( 60 )
, m_render_at_fps( false )
, m_locked( false )
{}

void RenderingState::addPlaneRenderer( const unsigned plane_index, const Array2s& r )
{
  m_plane_renderers.emplace_back( plane_index, r );
}

void RenderingState::addCylinderRenderer( const unsigned cylinder_index, const scalar& L )
{
  m_cylinder_renderers.emplace_back( cylinder_index, L );
}

void RenderingState::addPortalRenderer( const unsigned portal_index, const Array2s& half_width0, const Array2s& half_width1 )
{
  assert( ( half_width0 >= 0.0 ).all() );
  assert( ( half_width1 >= 0.0 ).all() );
  m_portal_renderers.emplace_back( portal_index, half_width0, half_width1 );
}

void RenderingState::setPerspectvieCameraSettings( const scalar& theta, const scalar& phi, const scalar& rho, const Vector3s& lookat, const Vector3s& up )
{
  m_use_perspective_camera = true;
  m_use_orthographic_camera = false;
  m_camera_theta = theta;
  m_camera_phi = phi;
  m_camera_rho = rho;
  m_camera_lookat = lookat;
  m_camera_up = up;
}

void RenderingState::setOrthographicCameraSettings( const std::string& projection_plane, const Vector3s& x, const scalar& scale )
{
  m_use_perspective_camera = false;
  m_use_orthographic_camera = true;
  m_ortho_projection_plane = projection_plane;
  m_ortho_x = x;
  m_ortho_scale = scale;
}

std::vector<PlaneRendererState>::size_type RenderingState::numPlaneRenderers() const
{
  return m_plane_renderers.size();
}

const PlaneRendererState& RenderingState::planeRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index ) const
{
  assert( plane_renderer_index < numPlaneRenderers() );
  return m_plane_renderers[plane_renderer_index];
}

PlaneRendererState& RenderingState::planeRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index )
{
  assert( plane_renderer_index < numPlaneRenderers() );
  return m_plane_renderers[plane_renderer_index];
}

std::vector<PlaneRendererState>& RenderingState::planeRenderers()
{
  return m_plane_renderers;
}

void RenderingState::sortPlaneRenderers()
{
  std::sort( m_plane_renderers.begin(), m_plane_renderers.end() );  
}

void RenderingState::deleteStaticPlaneRenderer( const std::vector<PlaneRendererState>::size_type plane_renderer_index )
{
  assert( plane_renderer_index < m_plane_renderers.size() );
  m_plane_renderers.erase( m_plane_renderers.begin() + plane_renderer_index );
}

std::vector<CylinderRendererState>::size_type RenderingState::numCylinderRenderers() const
{
  return m_cylinder_renderers.size();
}

const CylinderRendererState& RenderingState::cylinderRenderer( const std::vector<CylinderRendererState>::size_type cylinder_renderer_index ) const
{
  assert( cylinder_renderer_index < numCylinderRenderers() );
  return m_cylinder_renderers[cylinder_renderer_index];
}

std::vector<PlanarPortalRendererState>::size_type RenderingState::numPlanarPortalRenderers() const
{
  return m_portal_renderers.size();
}

const PlanarPortalRendererState& RenderingState::portalRenderer( const std::vector<PlanarPortalRendererState>::size_type portal_renderer_index ) const
{
  assert( portal_renderer_index < numPlanarPortalRenderers() );
  return m_portal_renderers[portal_renderer_index];
}

bool RenderingState::cameraSettingsInitialized() const
{
  return m_use_perspective_camera || m_use_orthographic_camera;
}

bool RenderingState::perspectiveCameraSelected() const
{
  return m_use_perspective_camera;
}

bool RenderingState::orthographicCameraSelected() const
{
  return m_use_orthographic_camera;
}

void RenderingState::setFPS( const unsigned fps )
{
  m_fps = fps;
}

void RenderingState::setRenderAtFPS( const bool render_at_fps )
{
  m_render_at_fps = render_at_fps;
}

void RenderingState::setLocked( const bool locked )
{
  m_locked = locked;
}

const Vector3s& RenderingState::cameraUp() const
{
  return m_camera_up;
}

const scalar& RenderingState::cameraTheta() const
{
  return m_camera_theta;
}

const scalar& RenderingState::cameraPhi() const
{
  return m_camera_phi;
}

const scalar& RenderingState::cameraRho() const
{
  return m_camera_rho;
}

const Vector3s& RenderingState::cameraLookAt() const
{
  return m_camera_lookat;
}

const std::string& RenderingState::orthographicProjectionPlane() const
{
  return m_ortho_projection_plane;
}

const Vector3s& RenderingState::orthographicX() const
{
  return m_ortho_x;
}

const scalar& RenderingState::orthographicScale() const
{
  return m_ortho_scale;
}

unsigned RenderingState::FPS() const
{
  return m_fps;
}

bool RenderingState::renderAtFPS() const
{
  return m_render_at_fps;
}

bool RenderingState::locked() const
{
  return m_locked;
}
