#include "Window.h"

#include <QMenu>
#include <QMenuBar>

#include "ContentWidget.h"

Window::Window( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent )
: QMainWindow( parent )
{
  ContentWidget* content_widget{ new ContentWidget{ scene_name, sim_settings, render_settings, this } };
  setCentralWidget( content_widget );

  // TODO: Instead of manual wiring, add actions to the content widget's buttons instead? Manual works fine for now though.

  // File menu actions
  {
    QMenu* file{ menuBar()->addMenu( tr( "File" ) ) };
    file->addAction( tr( "Open Sim..." ), content_widget, &ContentWidget::openUserScene, Qt::CTRL + Qt::Key_O );
    file->addAction( content_widget->reloadAction() );
    file->addSeparator();
    file->addAction( tr( "Save Image..." ), content_widget, &ContentWidget::exportImage, Qt::CTRL + Qt::Key_I );
    QAction* save_movie_action = file->addAction( tr( "Save Movie..." ), content_widget, &ContentWidget::exportMovie, Qt::CTRL + Qt::Key_M );
    save_movie_action->setCheckable( true );
    content_widget->wireSaveMovieAction( save_movie_action );
    #ifdef USE_HDF5
    QAction* save_state_action = file->addAction( tr( "Save State..." ), content_widget, &ContentWidget::exportState, Qt::CTRL + Qt::Key_S );
    save_state_action->setCheckable( true );
    content_widget->wireSaveStateAction( save_state_action );
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
