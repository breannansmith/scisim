#include "ContentWidget.h"

#include <QtGui>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QFileDialog>

#include <cassert>

#include "rigidbody2dutils/RigidBody2DSceneParser.h"

#include "GLWidget.h"

ContentWidget::ContentWidget( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent )
: QWidget( parent )
, m_idle_timer( nullptr )
, m_gl_widget( new GLWidget( this, QSurfaceFormat::defaultFormat() ) )
, m_xml_file_name()
{
  QVBoxLayout* mainLayout{ new QVBoxLayout };
  setLayout( mainLayout );

  // Add the OpenGL display
  mainLayout->addWidget( m_gl_widget );

  // Add the layout for controls
  {
    QGridLayout* controls_layout{ new QGridLayout };
    mainLayout->addLayout( controls_layout );

    // Solver buttons
    m_simulate_checkbox = new QCheckBox{ tr( "Simulate" ) };
    controls_layout->addWidget( m_simulate_checkbox, 0, 0 );
    m_simulate_checkbox->setChecked( false );
    connect( m_simulate_checkbox, &QCheckBox::toggled, this, &ContentWidget::simulateToggled );

    // Button for taking a single time step
    {
      QPushButton* step_button{ new QPushButton{ tr( "Step" ), this } };
      controls_layout->addWidget( step_button, 0, 1 );
      connect( step_button, &QPushButton::clicked, this, &ContentWidget::takeStep );
    }

    // Button for resetting the simulation
    {
      QPushButton* reset_button{ new QPushButton{ tr( "Reset" ), this } };
      controls_layout->addWidget( reset_button, 0, 2 );
      connect( reset_button, &QPushButton::clicked, this, &ContentWidget::resetSystem );
    }

    // Button for rendering at the specified FPS
    m_render_at_fps_checkbox = new QCheckBox{ tr( "Render FPS" ) };
    controls_layout->addWidget( m_render_at_fps_checkbox, 0, 3 );
    connect( m_render_at_fps_checkbox, &QCheckBox::toggled, this, &ContentWidget::renderAtFPSToggled );

    // Buttons for locking the camera controls
    m_lock_camera_button = new QCheckBox{ tr( "Lock Camera" ) };
    controls_layout->addWidget( m_lock_camera_button, 1, 0 );
    connect( m_lock_camera_button, &QCheckBox::toggled, this, &ContentWidget::lockCameraToggled );

    // Movie export controls
    m_export_movie_checkbox = new QCheckBox{ tr( "Export Movie" ) };
    controls_layout->addWidget( m_export_movie_checkbox, 1, 2 );
    m_export_movie_checkbox->setChecked( false );
    connect( m_export_movie_checkbox, &QCheckBox::toggled, this, &ContentWidget::exportMovieToggled );

    // Label for movie output FPS
    {
      QLabel* fps_label{ new QLabel{ this } };
      controls_layout->addWidget( fps_label, 1, 3 );
      fps_label->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
      fps_label->setText( tr( "FPS:" ) );
    }

    // Input for movie output FPS
    m_fps_spin_box = new QSpinBox{ this };
    controls_layout->addWidget( m_fps_spin_box, 1, 4 );
    m_fps_spin_box->setRange( 1, 1000 );
    m_fps_spin_box->setValue( 50 );
    connect( m_fps_spin_box, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ContentWidget::movieFPSChanged );
    m_gl_widget->setMovieFPS( m_fps_spin_box->value() );
  }

  // Create a timer that triggers simulation steps when Qt is idle
  m_idle_timer = new QTimer{ this };
  connect( m_idle_timer, &QTimer::timeout, this, &ContentWidget::takeStep );

  if( !scene_name.isEmpty() )
  {
    initializeSimulation( scene_name, false, sim_settings, render_settings );
  }

  this->setFocusPolicy( Qt::StrongFocus );
  this->setFocus();
}

void ContentWidget::keyPressEvent( QKeyEvent* event )
{
  assert( event != nullptr );

  if( event->key() == Qt::Key_Space )
  {
    toggleSimulationCheckbox();
  }
  else if( event->key() == Qt::Key_R )
  {
    resetSystem();
  }
  else if( event->key() == Qt::Key_S )
  {
    takeStep();
  }
}

void ContentWidget::toggleSimulationCheckbox()
{
  assert( m_simulate_checkbox != nullptr );
  m_simulate_checkbox->toggle();
}

void ContentWidget::disableMovieExport()
{
  assert( m_export_movie_checkbox != nullptr );
  m_export_movie_checkbox->setCheckState( Qt::Unchecked );
}

void ContentWidget::takeStep()
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->stepSystem();
}

void ContentWidget::resetSystem()
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->resetSystem();

  disableMovieExport();
}

void ContentWidget::openScene()
{
  // Obtain a file name from the user
  const QString xml_scene_file_name{ getOpenFileNameFromUser( tr( "Please Select a Scene File" ) ) };

  // Try to load the file
  openScene( xml_scene_file_name, true );
}

void ContentWidget::openScene( const QString& scene_file_name, const bool render_on_load )
{
  // If the user provided a valid file
  if( QFile::exists( scene_file_name ) )
  {
    SimSettings sim_settings;
    RenderSettings render_settings;
    const bool loaded{ RigidBody2DSceneParser::parseXMLSceneFile( scene_file_name.toStdString(), sim_settings, render_settings ) };
    if( !loaded )
    {
      qWarning() << "Failed to load file: " << scene_file_name;
      return;
    }

    initializeSimulation( scene_file_name, render_on_load, sim_settings, render_settings );
  }
  else
  {
    using str = std::string;
    str msg{ str{"Warning, requested file '"} + scene_file_name.toStdString() + str{"' does not exist."} };
    qWarning( "%s", msg.c_str() );
  }
}

void ContentWidget::initializeSimulation( const QString& scene_file_name, const bool render_on_load, SimSettings& sim_settings, RenderSettings& render_settings )
{
  // If the sample count changed, update the GL widget with a new format
  if( m_gl_widget->sampleCount() != render_settings.num_aa_samples )
  {
    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    format.setSamples( render_settings.num_aa_samples );

    GLWidget* new_gl_widget = new GLWidget( this, format );
    layout()->replaceWidget( m_gl_widget, new_gl_widget );
    m_gl_widget->deleteLater();
    m_gl_widget = new_gl_widget;
  }

  // Initialize the simulation
  unsigned fps = render_settings.fps;
  bool render_at_fps = render_settings.render_at_fps;
  bool lock_camera = render_settings.lock_camera;
  m_gl_widget->initializeSimulation( scene_file_name, render_on_load, sim_settings, render_settings );

  // Make sure the simulation is not running when we start
  assert( m_simulate_checkbox != nullptr );
  if( m_simulate_checkbox->isChecked() )
  {
    toggleSimulationCheckbox();
  }

  // Update UI elements
  assert( m_fps_spin_box != nullptr );
  m_fps_spin_box->setValue( fps );
  assert( m_render_at_fps_checkbox != nullptr );
  m_render_at_fps_checkbox->setCheckState( render_at_fps ? Qt::Checked : Qt::Unchecked );
  assert( m_lock_camera_button != nullptr );
  m_lock_camera_button->setCheckState( lock_camera ? Qt::Checked : Qt::Unchecked );

  m_xml_file_name = scene_file_name;

  disableMovieExport();
}

void ContentWidget::reloadScene()
{
  // Try to load the file
  openScene( m_xml_file_name, true );
}

void ContentWidget::simulateToggled( const bool state )
{
  assert( m_idle_timer != nullptr );

  // Start the idle timer
  if( state )
  {
    m_idle_timer->start( 0 );
  }
  // Stop the idle timer
  else
  {
    m_idle_timer->stop();
  }
}

void ContentWidget::renderAtFPSToggled( const bool render_at_fps )
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->renderAtFPS( render_at_fps );
}

void ContentWidget::lockCameraToggled( const bool lock_camera )
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->lockCamera( lock_camera );
}

void ContentWidget::exportMovieToggled( const bool checked )
{
  assert( m_gl_widget != nullptr );

  if( checked )
  {
    // Attempt to get a directory name
    const QString dir_name{ getDirectoryNameFromUser( tr( "Please Specify an Image Directory" ) ) };
    if( !dir_name.isEmpty() )
    {
      m_gl_widget->setMovieDir( dir_name );
    }
    else
    {
      assert( m_export_movie_checkbox != nullptr );
      m_export_movie_checkbox->toggle();
    }
  }
  else
  {
    m_gl_widget->setMovieDir( tr( "" ) );
  }
}

void ContentWidget::toggleHUD()
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->toggleHUD();
}

void ContentWidget::centerCamera()
{
  assert( m_gl_widget != nullptr );
  m_gl_widget->centerCamera();
}

void ContentWidget::exportImage()
{
  assert( m_gl_widget != nullptr );
  const QString file_name{ getSaveFileNameFromUser( tr( "Please Specify an Image Name" ) ) };
  if( !file_name.isEmpty() )
  {
    m_gl_widget->saveScreenshot( file_name );
  }
}

void ContentWidget::exportMovie()
{
  assert( m_export_movie_checkbox != nullptr );
  m_export_movie_checkbox->setChecked( false );
  m_export_movie_checkbox->setChecked( true );
}

void ContentWidget::movieFPSChanged( int fps )
{
  m_gl_widget->setMovieFPS( fps );
}

void ContentWidget::exportCameraSettings()
{
  m_gl_widget->exportCameraSettings();
}

QString ContentWidget::getOpenFileNameFromUser( const QString& prompt )
{
  const QString file_name{ QFileDialog::getOpenFileName( this, prompt ) };
  return file_name;
}

QString ContentWidget::getSaveFileNameFromUser( const QString& prompt )
{
  const QString file_name{ QFileDialog::getSaveFileName( this, prompt ) };
  return file_name;
}

QString ContentWidget::getDirectoryNameFromUser( const QString& prompt )
{
  const QString file_name{ QFileDialog::getExistingDirectory( this, prompt ) };
  return file_name;
}
