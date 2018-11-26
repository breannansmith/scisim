#include "Window.h"

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
    file->addAction( content_widget->openAction() );
    file->addAction( content_widget->reloadAction() );
    file->addSeparator();
    file->addAction( content_widget->saveImageAction() );
    file->addAction( content_widget->saveMovieAction() );
    #ifdef USE_HDF5
    file->addAction( content_widget->saveStateAction() );
    #endif
    file->addSeparator();
    file->addAction( content_widget->lockOutputFPSAction() );
  }

  // View menu actions
  {
    QMenu* view{ menuBar()->addMenu( tr( "View" ) ) };
    QAction* toggle_controls = view->addAction( tr( "Show Controls" ), content_widget, &ContentWidget::toggleControls, Qt::Key_U );
    toggle_controls->setCheckable( true );
    toggle_controls->setChecked( true );
    view->addAction( content_widget->displayHUDAction() );
    view->addSeparator();
    view->addAction( content_widget->centerCameraAction() );
    view->addAction( content_widget->lockCameraAction() );
    view->addSeparator();
    view->addAction( content_widget->lockRenderFPSAction() );
    view->addSeparator();
    view->addAction( content_widget->displayCameraAction() );
  }

  // Simulation menu actions
  {
    QMenu* sim{ menuBar()->addMenu( tr( "Simulation" ) ) };
    sim->addAction( content_widget->simulateAction() );
    sim->addAction( content_widget->resetAction() );
    sim->addAction( content_widget->stepAction() );
  }
}
