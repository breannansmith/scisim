#include "Window.h"

#include <cassert>
#include <QMenu>
#include <QMenuBar>
#include "ContentWidget.h"

Window::Window( const QString& scene_name, QWidget* parent )
: QMainWindow( parent )
, m_content_widget( nullptr )
{
  m_content_widget = new ContentWidget{ scene_name, this };

  QMenu* file{ menuBar()->addMenu( tr( "File" ) ) };

  QMenu* view{ menuBar()->addMenu( tr( "View" ) ) };
  
  QAction* seperator{ new QAction{ this } };
  seperator->setSeparator( true );

  // File menu actions

  // Load the input xml file
  QAction* open_scene{ new QAction{ tr( "Open..." ), this } };
  open_scene->setShortcut( tr( "Ctrl+o" ) );
  file->addAction( open_scene );
  connect( open_scene, SIGNAL( triggered() ), m_content_widget, SLOT( openScene() ) );

  // Reload the current xml file
  QAction* reload_scene{ new QAction{ tr( "Reload" ), this } };
  reload_scene->setShortcut( tr( "Ctrl+r" ) );
  file->addAction( reload_scene );
  connect( reload_scene, SIGNAL( triggered() ), m_content_widget, SLOT( reloadScene() ) );

  // Add a seperator
  file->addAction( seperator );

  // Export an image of the scene
  QAction* export_image{ new QAction{ tr( "Export Image..." ), this } };
  export_image->setShortcut( tr( "Ctrl+i" ) );
  file->addAction( export_image );
  connect( export_image, SIGNAL( triggered() ), m_content_widget, SLOT( exportImage() ) );

  // Export a movie of the scene
  QAction* export_movie{ new QAction{ tr( "Export Movie..." ), this } };
  export_movie->setShortcut( tr( "Ctrl+m" ) );
  file->addAction( export_movie );
  connect( export_movie, SIGNAL( triggered() ), m_content_widget, SLOT( exportMovie() ) );

  // Add a seperator
  QAction* seperator2{ new QAction( this ) };
  seperator2->setSeparator( true );
  file->addAction( seperator2 );

  // Export the current camera settings
  QAction* export_camera_settings{ new QAction{ tr( "Export Camera..." ), this } };
  file->addAction( export_camera_settings );
  connect( export_camera_settings, SIGNAL( triggered() ), m_content_widget, SLOT( exportCameraSettings() ) );

  // View menu actions

  // Toggle the heads up display
  QAction* toggle_hud{ new QAction{ tr( "Togge HUD" ), this } };
  toggle_hud->setShortcut( tr( "h" ) );
  view->addAction( toggle_hud );
  connect( toggle_hud, SIGNAL( triggered() ), m_content_widget, SLOT( toggleHUD() ) );

  // Toggle the grids that show the scale of objects
  QAction* toggle_xy_grid{ new QAction{ tr( "Togge XY Grid" ), this } };
  view->addAction( toggle_xy_grid );
  connect( toggle_xy_grid, SIGNAL( triggered() ), m_content_widget, SLOT( toggleXYGrid() ) );

  QAction* toggle_yz_grid{ new QAction{ tr( "Togge YZ Grid" ), this } };
  view->addAction( toggle_yz_grid );
  connect( toggle_yz_grid, SIGNAL( triggered() ), m_content_widget, SLOT( toggleYZGrid() ) );

  QAction* toggle_xz_grid{ new QAction{ tr( "Togge XZ Grid" ), this } };
  view->addAction( toggle_xz_grid );
  connect( toggle_xz_grid, SIGNAL( triggered() ), m_content_widget, SLOT( toggleXZGrid() ) );

  // Add a seperator
  view->addAction( seperator );

  // Center the camera
  QAction* center_camera{ new QAction{ tr( "Center Camera" ), this } };
  center_camera->setShortcut( tr( "c" ) );
  view->addAction( center_camera );
  connect( center_camera, SIGNAL( triggered() ), m_content_widget, SLOT( centerCamera() ) );

  // Perspective camera
  QAction* enable_perspective_camera{ new QAction{ tr( "Perspective Camera" ), this } };
  view->addAction( enable_perspective_camera );
  connect( enable_perspective_camera, SIGNAL( triggered() ), m_content_widget, SLOT( enablePerspectiveCamera() ) );

  // Orthographic projections
  QAction* enable_xy_orthographic_camera{ new QAction{ tr( "XY Orthographic Camera" ), this } };
  view->addAction( enable_xy_orthographic_camera );
  connect( enable_xy_orthographic_camera, SIGNAL( triggered() ), m_content_widget, SLOT( enableOrthographicXYCamera() ) );

  QAction* enable_zy_orthographic_camera{ new QAction{ tr( "ZY Orthographic Camera" ), this } };
  view->addAction( enable_zy_orthographic_camera );
  connect( enable_zy_orthographic_camera, SIGNAL( triggered() ), m_content_widget, SLOT( enableOrthographicZYCamera() ) );

  QAction* enable_zx_orthographic_camera{ new QAction{ tr( "ZX Orthographic Camera" ), this } };
  view->addAction( enable_zx_orthographic_camera );
  connect( enable_zx_orthographic_camera, SIGNAL( triggered() ), m_content_widget, SLOT( enableOrthographicZXCamera() ) );

  setCentralWidget( m_content_widget );
}

void Window::keyPressEvent( QKeyEvent* event )
{
  assert( event != nullptr );

  if( event->key() == Qt::Key_Space )
  {
    m_content_widget->toggleSimulationCheckbox();
  }
  else if( event->key() == Qt::Key_R )
  {
    m_content_widget->resetSystem();
  }
  else if( event->key() == Qt::Key_S )
  {
    m_content_widget->takeStep();
  }
  // TODO: Code up center camera ...
}

void Window::closeEvent( QCloseEvent* event )
{
  assert( m_content_widget != nullptr );
  m_content_widget->close();
}
