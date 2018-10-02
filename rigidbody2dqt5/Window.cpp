#include "Window.h"

#include <cassert>

#include <QMenu>
#include <QMenuBar>

#include "ContentWidget.h"

Window::Window( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent )
: QMainWindow( parent )
{
  ContentWidget* content_widget{ new ContentWidget{ scene_name, sim_settings, render_settings, this } };
  setCentralWidget( content_widget );

  // File menu actions
  {
    QMenu* file{ menuBar()->addMenu( tr( "File" ) ) };
    assert( file != nullptr );
    file->addAction( tr( "Open..." ), content_widget, SLOT( openScene() ), tr( "Ctrl+o" ) );
    file->addAction( tr( "Reload" ), content_widget, SLOT( reloadScene() ), tr( "Ctrl+r" ) );
    file->addSeparator();
    file->addAction( tr( "Export Image..." ), content_widget, SLOT( exportImage() ), tr( "Ctrl+i" ) );
    file->addAction( tr( "Export Movie..." ), content_widget, SLOT( exportMovie() ), tr( "Ctrl+m" ) );
    file->addSeparator();
    file->addAction( tr( "Export Camera..." ), content_widget, SLOT( exportCameraSettings() ) );
  }

  // View menu actions
  {
    QMenu* view{ menuBar()->addMenu( tr( "View" ) ) };
    assert( view != nullptr );
    view->addAction( tr( "Togge HUD" ), content_widget, SLOT( toggleHUD() ), tr( "h" ) );
    view->addSeparator();
    view->addAction( tr( "Center Camera" ), content_widget, SLOT( centerCamera() ), tr( "c" ) );
  }
}
