#include "Window.h"

#include <cassert>

#include <QMenu>
#include <QMenuBar>
#include <QKeyEvent>

#include "ContentWidget.h"

Window::Window( const QString& scene_name, QWidget* parent )
: QMainWindow( parent )
, m_content_widget( nullptr )
{
  m_content_widget = new ContentWidget{ scene_name, this };

  QMenu* file{ menuBar()->addMenu( tr( "File" ) ) };

  QMenu* view{ menuBar()->addMenu( tr( "View" ) ) };
  
  QAction* separator{ new QAction{ this } };
  separator->setSeparator( true );

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

  // Add a separator
  file->addAction( separator );

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

  // Add a separator
  QAction* separator2{ new QAction{ this } };
  separator2->setSeparator( true );
  file->addAction( separator2 );
  
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

  // Add a separator
  view->addAction( separator );

  // Center the camera
  QAction* center_camera{ new QAction( tr( "Center Camera" ), this ) };
  center_camera->setShortcut( tr( "c" ) );
  view->addAction( center_camera );
  connect( center_camera, SIGNAL( triggered() ), m_content_widget, SLOT( centerCamera() ) );

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
}

void Window::closeEvent( QCloseEvent* event )
{
  assert( m_content_widget != nullptr );
  m_content_widget->close();
}
