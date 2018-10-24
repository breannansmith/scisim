#include <QApplication>
#include <QDesktopWidget>
#include <QSurfaceFormat>

#ifdef USE_MACOS
#include <QRegularExpression>
#endif

#include <cassert>

#include "Window.h"

#include "ball2dutils/Ball2DSceneParser.h"

#ifdef USE_PYTHON
#include <Python.h>
#include "ball2d/PythonScripting.h"
#include "scisim/PythonTools.h"
#endif

#ifdef USE_MACOS
static bool isProcessSerialNumber( char* text )
{
  QRegularExpression psnPattern("^-psn_\\d+_\\d+$");
  return psnPattern.match(text).hasMatch();
}
#endif

static void centerWindow( Window& window )
{
  const QDesktopWidget* const desktop{ QApplication::desktop() };
  assert( desktop != nullptr );

  const int screen_width{ desktop->screenGeometry().width() };
  const int screen_height{ desktop->screenGeometry().height() };

  const QSize window_size{ window.size() };
  const int width{ window_size.width() };
  const int height{ window_size.height() };

  const int x{ ( screen_width - width ) / 2 };
  const int y{ ( screen_height - height ) / 2 };

  window.move( x, y );
}

#ifdef USE_PYTHON
static void exitCleanup()
{
  Py_Finalize();
}
#endif

int main( int argc, char** argv )
{
  if( argc > 2 )
  {
    qWarning( "Error, must provide a valid configuration file name or no argument. Exiting." );
    return EXIT_FAILURE;
  }

  SimSettings sim_settings;
  RenderSettings render_settings;
  #ifdef USE_MACOS
  if( argc == 2 && !isProcessSerialNumber( argv[1] ) )
  #else
  if( argc == 2 )
  #endif
  {
    const bool loaded{ Ball2DSceneParser::parseXMLSceneFile( argv[1], sim_settings, render_settings ) };
    if( !loaded )
    {
      qWarning( "Error, failed to parse input file." );
      return EXIT_FAILURE;
    }
  }

  #ifdef USE_PYTHON
  // Initialize the Python interpreter
  Py_SetProgramName( argv[0] );
  Py_Initialize();

  // Initialize a callback that will close down the interpreter
  atexit( exitCleanup );

  // Allow subsequent Python commands to use the sys module
  PythonTools::pythonCommand( "import sys" );

  // Prevent Python from intercepting the interrupt signal
  PythonTools::pythonCommand( "import signal" );
  PythonTools::pythonCommand( "signal.signal( signal.SIGINT, signal.SIG_DFL )" );

  // Initialize the callbacks
  PythonScripting::initializeCallbacks();
  #endif

  {
    QSurfaceFormat format;
    format.setSamples( 4 );
    format.setVersion( 3, 3 );
    format.setProfile( QSurfaceFormat::CoreProfile );
    QSurfaceFormat::setDefaultFormat( format );
  }

  QApplication app{ argc, argv };

  Window window{ argc == 2 ? argv[1] : "", sim_settings, render_settings };
  window.resize( window.sizeHint() );
  window.setWindowTitle( QObject::tr("SCISim 2D Ball Simulation") );
  centerWindow( window );
  window.show();
  window.raise();

  return app.exec();
}
