#include "ContentWidget.h"

#include <QApplication>
#include <QCheckBox>
#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QtGui>
#include <QVBoxLayout>

#include <cassert>

#include "ball2dutils/Ball2DSceneParser.h"

#include "GLWidget.h"
#include "SimWorker.h"

// !!!! TODO: DELETE THESE AFTER TESTING
#include <iostream>

ContentWidget::ContentWidget( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent )
: QWidget( parent )
, m_gl_widget( new GLWidget( this, QSurfaceFormat::defaultFormat() ) )
, m_simulate_checkbox( nullptr )
, m_render_at_fps_checkbox( nullptr )
, m_lock_camera_button( nullptr )
, m_export_movie_checkbox( nullptr )
, m_fps_spin_box( nullptr )
, m_step_button( nullptr )
, m_reset_button( nullptr )
, m_sim_thread()
, m_sim_worker( nullptr )
, m_xml_file_name()
, m_simulate_toggled( false )
, m_render_at_fps( false )
, m_ball_colors()
, m_color_gen( 0.0, 1.0 )
, m_ball_color_gen( 1337 )
, m_movie_dir_name()
, m_movie_dir()
, m_output_fps( 30 )
, m_plane_render_settings0()
, m_drum_render_settings0()
, m_portal_render_settings0()
, m_plane_render_settings()
, m_drum_render_settings()
, m_portal_render_settings()
, m_empty( true )
, m_bbox()
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
    m_step_button = new QPushButton{ tr( "Step" ), this };
    controls_layout->addWidget( m_step_button, 0, 1 );

    // Button for resetting the simulation
    m_reset_button = new QPushButton{ tr( "Reset" ), this };
    controls_layout->addWidget( m_reset_button, 0, 2 );

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
    m_fps_spin_box->setValue( 30 );
    connect( m_fps_spin_box, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &ContentWidget::setMovieFPS );
  }

  m_sim_worker = new SimWorker();
  // TODO: Make SimWorker take the init stuff as parameters
  if( !scene_name.isEmpty() )
  {
    initializeSimAndGL( scene_name, false, sim_settings, render_settings, *m_sim_worker );
  }

  m_sim_thread.start();

  m_sim_worker->moveToThread( &m_sim_thread );

  wireSimWorker();

  setMovieFPS( m_fps_spin_box->value() );

  this->setFocusPolicy( Qt::StrongFocus );
  this->setFocus();
}

void ContentWidget::wireSimWorker()
{
  connect( &m_sim_thread, &QThread::finished, m_sim_worker, &QObject::deleteLater );
  connect( this, &ContentWidget::stepSimulation, m_sim_worker, &SimWorker::takeStep );
  connect( m_step_button, &QPushButton::clicked, m_sim_worker, &SimWorker::takeStep );
  connect( this, &ContentWidget::resetSimulation, m_sim_worker, &SimWorker::reset );
  connect( m_reset_button, &QPushButton::clicked, m_sim_worker, &SimWorker::reset );
  connect( this, &ContentWidget::outputFPSChanged, m_sim_worker, &SimWorker::setOutputFPS );
  connect( m_sim_worker, &SimWorker::postStep, this, &ContentWidget::copyStepResults, Qt::BlockingQueuedConnection );
  connect( this, &ContentWidget::exportEnabled, m_sim_worker, &SimWorker::exportMovieInit );
}

ContentWidget::~ContentWidget()
{
  // The worker could still be running, so stop it from sending any new events to this object
  m_sim_worker->disconnect();
  // If, in the vanishingly rare case that the worker posted an event after event processing ceased
  // but before disconnect, just flush the event queue
  QCoreApplication::removePostedEvents( this );
  // The worker thread could have pending step events, which we no longer care about,
  // so flush them
  QCoreApplication::removePostedEvents( m_sim_worker );
  // Tell the sim thread to exit and wait for all running tasks to complete
  m_sim_thread.quit();
  m_sim_thread.wait();
}

void ContentWidget::keyPressEvent( QKeyEvent* event )
{
  assert( event != nullptr );

  if( event->key() == Qt::Key_Space )
  {
    m_simulate_checkbox->toggle();
  }
  else if( event->key() == Qt::Key_R )
  {
    emit resetSimulation();
  }
  else if( event->key() == Qt::Key_S )
  {
    emit stepSimulation();
  }
}

void ContentWidget::disableMovieExport()
{
  assert( m_export_movie_checkbox != nullptr );
  m_export_movie_checkbox->setCheckState( Qt::Unchecked );
  setMovieDir( tr( "" ) );
}

void ContentWidget::insertBallCallback( const int num_balls )
{
  m_ball_colors.conservativeResize( 3 * num_balls );
  m_ball_colors.segment<3>( 3 * num_balls - 3) = generateColor();
}

static void ballInsertCallback( void* context, int num_balls )
{
  static_cast<ContentWidget*>(context)->insertBallCallback(num_balls);
}

void ContentWidget::deletePlaneCallback( const int plane_idx )
{
  // For each plane renderer
  for( int rndr_idx = 0; rndr_idx < int(m_plane_render_settings.size()); rndr_idx++ )
  {
    PlaneRenderSettings& settings = m_plane_render_settings[rndr_idx];
    // Flag the renderer for deletion if it matches the marked plane
    if( settings.idx == plane_idx )
    {
      settings.idx = -1;
    }
    // Otherwise, if the index is greater than the marked plane, decrement the index
    else if( settings.idx > plane_idx )
    {
      settings.idx--;
    }
  }
  // Delete any flagged renderers
  m_plane_render_settings.erase(
    std::remove_if( m_plane_render_settings.begin(), m_plane_render_settings.end(),
      []( const PlaneRenderSettings& settings ){ return settings.idx == -1; } ),
    m_plane_render_settings.end() );
}

static void planeDeleteCallback( void* context, int plane_idx )
{
  static_cast<ContentWidget*>(context)->deletePlaneCallback(plane_idx);
}

void ContentWidget::copyStepResults( const bool was_reset, const bool fps_multiple, const int output_num )
{
  if( !was_reset )
  {
    if( !m_render_at_fps || fps_multiple )
    {
      m_empty = m_sim_worker->sim().state().empty();
      m_bbox = m_sim_worker->sim().state().computeBoundingBox();
      m_gl_widget->copyRenderState( m_sim_worker->sim().state(), m_ball_colors, m_plane_render_settings, m_drum_render_settings,
                                    m_portal_render_settings, scalar(m_sim_worker->integrator().dt()) * m_sim_worker->iteration(),
                                    m_sim_worker->endTime(), m_sim_worker->deltaH0(), m_sim_worker->deltap0(), m_sim_worker->deltaL0() );
      m_gl_widget->update();
    }

    if( !m_movie_dir_name.isEmpty() && fps_multiple )
    {
      QString output_image_name{ QString{ tr( "frame%1.png" ) }.arg( output_num, 10, 10, QLatin1Char('0') ) };
      m_gl_widget->saveScreenshot( m_movie_dir.filePath( output_image_name ) );
    }
  }
  else
  {
    // Reset the output movie option
    disableMovieExport();

    // Register UI callbacks for Python scripting
    m_sim_worker->scripting().registerBallInsertCallback( this, &ballInsertCallback );
    m_sim_worker->scripting().registerPlaneDeleteCallback( this, &planeDeleteCallback );

    m_plane_render_settings = m_plane_render_settings0;
    m_drum_render_settings = m_drum_render_settings0;
    m_portal_render_settings = m_portal_render_settings0;

    // Reset ball colors, in case the number of balls changed
    m_ball_color_gen = std::mt19937_64( 1337 );
    m_ball_colors.resize( 3 * m_sim_worker->sim().state().nballs() );
    for( int i = 0; i < m_ball_colors.size(); i += 3 )
    {
      m_ball_colors.segment<3>( i ) = generateColor();
    }

    m_empty = m_sim_worker->sim().state().empty();
    m_bbox = m_sim_worker->sim().state().computeBoundingBox();
    m_gl_widget->copyRenderState( m_sim_worker->sim().state(), m_ball_colors, m_plane_render_settings, m_drum_render_settings,
                                  m_portal_render_settings, scalar(m_sim_worker->integrator().dt()) * m_sim_worker->iteration(),
                                  m_sim_worker->endTime(), m_sim_worker->deltaH0(), m_sim_worker->deltap0(), m_sim_worker->deltaL0() );
    m_gl_widget->update();
  }

  if( m_simulate_toggled )
  {
    emit stepSimulation();
  }
}

void ContentWidget::openScene()
{
  // Obtain a file name from the user
  const QString xml_scene_file_name{ getOpenFileNameFromUser( tr( "Please Select a Scene File" ) ) };

  // Try to load the file
  if( !xml_scene_file_name.isEmpty() )
  {
    openScene( xml_scene_file_name, true );
  }
}

void ContentWidget::openScene( const QString& scene_file_name, const bool render_on_load )
{
  // If the user provided a valid file
  if( QFile::exists( scene_file_name ) )
  {
    // Attempt to load the file
    SimSettings sim_settings;
    RenderSettings render_settings;
    const bool loaded{ Ball2DSceneParser::parseXMLSceneFile( scene_file_name.toStdString(), sim_settings, render_settings ) };
    if( !loaded )
    {
      qWarning() << "Failed to load file: " << scene_file_name;
      return;
    }

    std::cout << "Swapping worker..." << std::endl;
    // Ignore any signals sent from the old sim worker
    std::cout << "   Disconnecting slots" << std::endl;
    m_sim_worker->disconnect();
    // Don't deliver any new signals to the old sim worker
    std::cout << "   Disconnecting signals" << std::endl;
    this->disconnect( m_sim_worker );
    // Clear any queued events on the worker we are deleting
    std::cout << "   Flushing step queue" << std::endl;
    QCoreApplication::removePostedEvents( m_sim_worker );
    // Schedule the old sim worker for deletion
    std::cout << "   Scheduling deletion" << std::endl;
    QMetaObject::invokeMethod( m_sim_worker, "deleteLater", Qt::QueuedConnection );

    std::cout << "   Creating a new worker" << std::endl;
    m_sim_worker = new SimWorker();
    initializeSimAndGL( scene_file_name, render_on_load, sim_settings, render_settings, *m_sim_worker );
    m_sim_worker->moveToThread( &m_sim_thread );

    wireSimWorker();

    setMovieFPS( m_fps_spin_box->value() );
    std::cout << "   done!" << std::endl;
  }
  else
  {
    using str = std::string;
    str msg{ str{"Warning, requested file '"} + scene_file_name.toStdString() + str{"' does not exist."} };
    qWarning( "%s", msg.c_str() );
  }
}

static int computeTimestepDisplayPrecision( const Rational<std::intmax_t>& dt, const std::string& dt_string )
{
  if( dt_string.find( '.' ) != std::string::npos )
  {
    return int( StringUtilities::computeNumCharactersToRight( dt_string, '.' ) );
  }
  else
  {
    std::string converted_dt_string;
    std::stringstream ss;
    ss << std::fixed << scalar( dt );
    ss >> converted_dt_string;
    return int( StringUtilities::computeNumCharactersToRight( converted_dt_string, '.' ) );
  }
}

// TODO: Rename this to initialize UI and GL
void ContentWidget::initializeSimAndGL( const QString& scene_file_name, const bool render_on_load, SimSettings& sim_settings,
                                        RenderSettings& render_settings, SimWorker& sim_worker )
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

  initializeSimulation( scene_file_name, sim_settings, render_settings, sim_worker );

  const int dt_display_precision = computeTimestepDisplayPrecision( sim_worker.integrator().dt(), sim_settings.dt_string );

  m_empty = sim_worker.sim().state().empty();
  m_bbox = sim_worker.sim().state().computeBoundingBox();
  m_gl_widget->initialize( render_on_load, render_settings, dt_display_precision, sim_worker.sim().state(), m_ball_colors,
                            m_plane_render_settings, m_drum_render_settings, m_portal_render_settings, sim_worker.endTime(),
                            m_empty, m_bbox );

  // Make sure the simulation is not running when we start
  assert( m_simulate_checkbox != nullptr );
  if( m_simulate_checkbox->isChecked() )
  {
    m_simulate_checkbox->toggle();
  }

  // Update UI elements
  assert( m_fps_spin_box != nullptr );
  m_fps_spin_box->setValue( render_settings.fps );
  assert( m_render_at_fps_checkbox != nullptr );
  m_render_at_fps_checkbox->setCheckState( render_settings.render_at_fps ? Qt::Checked : Qt::Unchecked );
  assert( m_lock_camera_button != nullptr );
  m_lock_camera_button->setCheckState( render_settings.lock_camera ? Qt::Checked : Qt::Unchecked );

  m_xml_file_name = scene_file_name;

  disableMovieExport();
}

void ContentWidget::reloadScene()
{
  if( !m_xml_file_name.isEmpty() )
  {
    openScene( m_xml_file_name, true );
  }
}

void ContentWidget::simulateToggled( const bool state )
{
  m_simulate_toggled = state;
  if( m_simulate_toggled )
  {
    emit stepSimulation();
  }
}

void ContentWidget::renderAtFPSToggled( const bool render_at_fps )
{
  m_render_at_fps = render_at_fps;
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
      setMovieDir( dir_name );
    }
    else
    {
      assert( m_export_movie_checkbox != nullptr );
      m_export_movie_checkbox->toggle();
    }
  }
  else
  {
    setMovieDir( tr( "" ) );
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
  m_gl_widget->centerCamera( true, m_empty, m_bbox );
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

void ContentWidget::exportCameraSettings()
{
  m_gl_widget->exportCameraSettings( m_output_fps, m_render_at_fps );
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

void ContentWidget::setMovieFPS( const int fps )
{
  assert( fps > 0 );
  m_output_fps = fps;

  disableMovieExport();

  if( m_simulate_checkbox->isChecked() )
  {
    m_simulate_checkbox->toggle();
  }

  emit outputFPSChanged( m_output_fps );
}

Vector3s ContentWidget::generateColor()
{
  Vector3s color( 1.0, 1.0, 1.0 );
  // Generate colors until we get one with a luminance in [0.1, 0.9]
  while( ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) > 0.9 ||
         ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) < 0.1 )
  {
    color.x() = m_color_gen( m_ball_color_gen );
    color.y() = m_color_gen( m_ball_color_gen );
    color.z() = m_color_gen( m_ball_color_gen );
  }
  return color;
}

static std::string xmlFilePath( const std::string& xml_file_name )
{
  std::string path;
  std::string file_name;
  StringUtilities::splitAtLastCharacterOccurence( xml_file_name, path, file_name, '/' );
  if( file_name.empty() )
  {
    using std::swap;
    swap( path, file_name );
  }
  return path;
}

// TODO: Move this into SimWorker and rename it
void ContentWidget::initializeSimulation( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings, SimWorker& sim_worker )
{
  // Push the initial state and cache it to allow resets
  sim_worker.sim().state() = std::move( sim_settings.state );
  sim_worker.sim().clearConstraintCache();
  sim_worker.sim0() = sim_worker.sim();

  // Push the initial integrator state and cache it to allow resets
  sim_worker.integrator0() = sim_settings.integrator;
  sim_worker.integrator() = sim_worker.integrator0();

  // Initialize the scripting callback
  {
    PythonScripting new_scripting{ xmlFilePath( xml_scene_file_name.toStdString() ), sim_settings.scripting_callback_name };
    swap( sim_worker.scripting(), new_scripting );
  }

  // Save the time and iteration related quantities
  sim_worker.iteration() = 0;
  sim_worker.endTime() = sim_settings.end_time;
  assert( sim_worker.endTime() > 0.0 );

  // Update the FPS setting
  if( render_settings.camera_set )
  {
    m_render_at_fps = render_settings.render_at_fps;
    m_output_fps = render_settings.fps;
  }
  assert( m_output_fps > 0 );
  setMovieFPS( m_output_fps );

  // Compute the initial energy, momentum, and angular momentum
  sim_worker.H0() = sim_worker.sim().state().computeTotalEnergy();
  sim_worker.p0() = sim_worker.sim().state().computeMomentum();
  sim_worker.L0() = sim_worker.sim().state().computeAngularMomentum();
  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  sim_worker.deltaH0() = 0.0;
  sim_worker.deltap0() = Vector2s::Zero();
  sim_worker.deltaL0() = 0.0;

  // Generate a random color for each ball
  m_ball_color_gen = std::mt19937_64( 1337 );
  m_ball_colors.resize( 3 * sim_worker.sim().state().nballs() );
  for( int i = 0; i < m_ball_colors.size(); i += 3 )
  {
    m_ball_colors.segment<3>( i ) = generateColor();
  }

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};

  // User-provided start of simulation python callback
  sim_worker.scripting().setState( sim_worker.sim().state() );
  sim_worker.scripting().startOfSimCallback();
  sim_worker.scripting().forgetState();

  // Register UI callbacks for Python scripting
  sim_worker.scripting().registerBallInsertCallback( this, &ballInsertCallback );
  sim_worker.scripting().registerPlaneDeleteCallback( this, &planeDeleteCallback );

  m_plane_render_settings = std::move( render_settings.plane_render_settings );
  m_plane_render_settings0 = m_plane_render_settings;
  m_drum_render_settings = std::move( render_settings.drum_render_settings );
  m_drum_render_settings0 = m_drum_render_settings;
  m_portal_render_settings = std::move( render_settings.portal_render_settings );
  m_portal_render_settings0 = m_portal_render_settings;
}

void ContentWidget::setMovieDir( const QString& dir_name )
{
  m_movie_dir_name = dir_name;

  if( !m_movie_dir_name.isEmpty() )
  {
    m_movie_dir.setPath( m_movie_dir_name );
    assert( m_movie_dir.exists() );
    emit exportEnabled();
  }
  else
  {
    m_movie_dir = QDir{};
  }
}
