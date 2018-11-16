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

  // TODO: Instead of manual wiring, add actions to the content widget's buttons instead? Manual works fine for now though.

  // File menu actions
  {
    QMenu* file{ menuBar()->addMenu( tr( "File" ) ) };
    assert( file != nullptr );
    file->addAction( tr( "Open Sim..." ), content_widget, &ContentWidget::openUserScene, Qt::CTRL + Qt::Key_O );
    file->addAction( tr( "Reload Sim" ), content_widget, &ContentWidget::reloadScene, Qt::CTRL + Qt::Key_R );
    file->addSeparator();
    file->addAction( tr( "Save Image..." ), content_widget, &ContentWidget::exportImage, Qt::CTRL + Qt::Key_I );
    QAction* movie_action = file->addAction( tr( "Save Movie..." ), content_widget, &ContentWidget::exportMovie, Qt::CTRL + Qt::Key_M );
    movie_action->setCheckable( true );
    content_widget->wireMovieAction( movie_action );
    file->addSeparator();
    file->addAction( tr( "Print Camera" ), content_widget, &ContentWidget::exportCameraSettings, Qt::CTRL + Qt::Key_C );
  }

  // View menu actions
  {
    QMenu* view{ menuBar()->addMenu( tr( "View" ) ) };
    assert( view != nullptr );
    QAction* toggle_controls = view->addAction( tr( "Show Controls" ), content_widget, &ContentWidget::toggleControls, Qt::Key_U );
    toggle_controls->setCheckable( true );
    toggle_controls->setChecked( true );
    QAction* toggle_hud = view->addAction( tr( "Show HUD" ), content_widget, &ContentWidget::toggleHUDCheckbox, Qt::Key_H );
    toggle_hud->setCheckable( true );
    toggle_hud->setChecked( true );
    content_widget->wireToggleHUD( toggle_hud );
    view->addSeparator();
    view->addAction( tr( "Center Camera" ), content_widget, &ContentWidget::centerCamera, Qt::Key_C );
    QAction* toggle_camera_lock = view->addAction( tr( "Lock Camera" ), content_widget, &ContentWidget::toggleCameraLock, Qt::Key_L );
    toggle_camera_lock->setCheckable( true );
    toggle_camera_lock->setChecked( content_widget->isCameraLocked() );
    content_widget->wireCameraLocked( toggle_camera_lock );
    view->addSeparator();
    QAction* toggle_fps_lock = view->addAction( tr( "Lock Render FPS" ), content_widget, &ContentWidget::toggleFPSLock, Qt::Key_F );
    toggle_fps_lock->setCheckable( true );
    toggle_fps_lock->setChecked( content_widget->isFPSLocked() );
    content_widget->wireFPSLocked( toggle_fps_lock );
  }

  // Simulation menu actions
  {
    QMenu* sim{ menuBar()->addMenu( tr( "Simulation" ) ) };
    assert( sim != nullptr );
    QAction* run = sim->addAction( tr( "Run Sim" ), content_widget, &ContentWidget::toggleSimulating, Qt::Key_Space );
    run->setCheckable( true );
    content_widget->wireRunSim( run );
    sim->addAction( tr( "Reset Sim" ), content_widget, &ContentWidget::callReset, Qt::Key_R );
    sim->addAction( tr( "Step Sim" ), content_widget, &ContentWidget::callStep, Qt::Key_S );
  }
}
