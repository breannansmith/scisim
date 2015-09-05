#include <QApplication>
#include <QDesktopWidget>

#include "Window.h"

#ifdef USE_PYTHON
#include <Python.h>
#include "Ball2D/PythonScripting.cpp"
#include "SCISim/PythonTools.h"
#endif

static void centerWindow( Window& window )
{
  const QDesktopWidget* const desktop{ QApplication::desktop() };

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

  QApplication app( argc, argv );
  Window window;
  window.resize( window.sizeHint() );
  window.setWindowTitle( "2D Ball Simulation" );
  centerWindow( window );
  window.show();
  window.raise();
  return app.exec();
}

